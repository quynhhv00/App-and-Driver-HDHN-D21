#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/mutex.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bach");
MODULE_DESCRIPTION("Character Device Driver for DHT11 (BBB)");
MODULE_VERSION("1.1");

#define DEVICE_NAME "dht11_driver"
#define CLASS_NAME  "dht_class"
#define BUFFER_SIZE 64
#define GPIO1_BASE   0x4804C000
#define GPIO_SIZE    0x1000
#define GPIO_OE      0x134
#define GPIO_DATAIN  0x138
#define GPIO_DATAOUT 0x13C
#define GPIO_DHT11   28   // GPIO1_28
#define TIMEOUT_US   100  // Timeout cho mỗi lần chờ (micro giây)

static int major_number;
static struct class *dev_class = NULL;
static struct device *dev_device = NULL;
static void __iomem *gpio_base = NULL;
static char result_buffer[BUFFER_SIZE];
static DEFINE_MUTEX(dht11_mutex);

static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char __user *, size_t, loff_t *);

static struct file_operations fops = {
    .open = device_open,
    .release = device_release,
    .read = device_read,
};

/* Đặt GPIO là output */
static void dht11_set_output(void) {
    u32 val = ioread32(gpio_base + GPIO_OE);
    val &= ~(1 << GPIO_DHT11);
    iowrite32(val, gpio_base + GPIO_OE);
}

/* Đặt GPIO là input */
static void dht11_set_input(void) {
    u32 val = ioread32(gpio_base + GPIO_OE);
    val |= (1 << GPIO_DHT11);
    iowrite32(val, gpio_base + GPIO_OE);
}

/* Ghi giá trị GPIO */
static void gpio_write(int value) {
    u32 val = ioread32(gpio_base + GPIO_DATAOUT);
    if (value)
        val |= (1 << GPIO_DHT11);
    else
        val &= ~(1 << GPIO_DHT11);
    iowrite32(val, gpio_base + GPIO_DATAOUT);
}

/* Đọc giá trị GPIO */
static int gpio_read(void) {
    u32 val = ioread32(gpio_base + GPIO_DATAIN);
    return (val & (1 << GPIO_DHT11)) ? 1 : 0;
}

/* Hàm chờ GPIO với timeout */
static int wait_gpio(int value, unsigned long timeout_us) {
    unsigned long start = jiffies;
    while (gpio_read() != value) {
        if (time_after(jiffies, start + usecs_to_jiffies(timeout_us))) {
            printk(KERN_ERR "DHT11: Timeout waiting for GPIO %d to be %d\n", GPIO_DHT11, value);
            return -ETIMEDOUT;
        }
        udelay(1);
    }
    return 0;
}

/* Hàm đọc dữ liệu thô từ DHT11 */
static int dht11_read_raw(u8 data[5]) {
    int i, j;
    u8 byte = 0;

    mutex_lock(&dht11_mutex);
    memset(data, 0, 5);

    printk(KERN_INFO "DHT11: Starting communication\n");

    /* Kiểm tra trạng thái GPIO ban đầu */
    dht11_set_input();
    if (!gpio_read()) {
        mutex_unlock(&dht11_mutex);
        printk(KERN_ERR "DHT11: GPIO %d stuck LOW, possible hardware issue\n", GPIO_DHT11);
        return -EIO;
    }

    /* Start signal */
    dht11_set_output();
    gpio_write(0);
    mdelay(18);
    gpio_write(1);
    udelay(30);

    dht11_set_input();

    /* Wait for DHT11 response */
    if (wait_gpio(0, 80)) {
        mutex_unlock(&dht11_mutex);
        printk(KERN_ERR "DHT11: No response (LOW)\n");
        return -ETIMEDOUT;
    }
    udelay(80);
    if (wait_gpio(1, 80)) {
        mutex_unlock(&dht11_mutex);
        printk(KERN_ERR "DHT11: No response (HIGH)\n");
        return -ETIMEDOUT;
    }
    udelay(80);

    /* Read 40 bits */
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            if (wait_gpio(1, 50)) {
                mutex_unlock(&dht11_mutex);
                printk(KERN_ERR "DHT11: Timeout waiting for HIGH bit\n");
                return -ETIMEDOUT;
            }
            udelay(30);
            byte = (byte << 1) | (gpio_read() ? 1 : 0);
            if (wait_gpio(0, 50)) {
                mutex_unlock(&dht11_mutex);
                printk(KERN_ERR "DHT11: Timeout waiting for LOW bit\n");
                return -ETIMEDOUT;
            }
        }
        data[i] = byte;
    }

    /* Checksum */
    if ((u8)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        mutex_unlock(&dht11_mutex);
        printk(KERN_ERR "DHT11: Checksum error\n");
        return -EIO;
    }

    mutex_unlock(&dht11_mutex);
    printk(KERN_INFO "DHT11: Read successful\n");
    return 0;
}

static int __init device_init(void) {
    printk(KERN_INFO "Initializing DHT11 Driver\n");

    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ERR "DHT11: Failed to register major number\n");
        return major_number;
    }

    dev_class = class_create(CLASS_NAME);
    if (IS_ERR(dev_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(dev_class);
    }

    dev_device = device_create(dev_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(dev_device)) {
        class_destroy(dev_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(dev_device);
    }

    gpio_base = ioremap(GPIO1_BASE, GPIO_SIZE);
    if (!gpio_base) {
        device_destroy(dev_class, MKDEV(major_number, 0));
        class_destroy(dev_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "DHT11: Failed to map GPIO memory\n");
        return -ENOMEM;
    }

    /* Đặt GPIO làm input ban đầu */
    dht11_set_input();
    printk(KERN_INFO "DHT11 Driver loaded successfully\n");
    return 0;
}

static void __exit device_exit(void) {
    iounmap(gpio_base);
    device_destroy(dev_class, MKDEV(major_number, 0));
    class_destroy(dev_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_INFO "DHT11 Driver unloaded\n");
}

static int device_open(struct inode *inodep, struct file *filep) {
    printk(KERN_INFO "DHT11 device opened\n");
    return 0;
}

static int device_release(struct inode *inodep, struct file *filep) {
    printk(KERN_INFO "DHT11 device closed\n");
    return 0;
}

static ssize_t device_read(struct file *filep, char __user *buffer, size_t len, loff_t *offset) {
    u8 data[5];
    int temp, hum, ret;
    int out_len;

    if (*offset > 0)
        return 0;

    ret = dht11_read_raw(data);
    if (ret < 0) {
        snprintf(result_buffer, BUFFER_SIZE, "Error reading DHT11 (%d)\n", ret);
    } else {
        hum = data[0];
        temp = data[2];
        snprintf(result_buffer, BUFFER_SIZE, "Temp: %dC, Hum: %d%%\n", temp, hum);
    }

    out_len = strlen(result_buffer);
    if (copy_to_user(buffer, result_buffer, out_len) != 0)
        return -EFAULT;

    *offset += out_len;
    return out_len;
}

module_init(device_init);
module_exit(device_exit);