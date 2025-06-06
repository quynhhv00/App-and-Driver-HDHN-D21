#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/io.h>         // ioremap
#include <linux/cdev.h>
#include <linux/delay.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bach");
MODULE_DESCRIPTION("Character Device Driver with 3 GPIO LEDs control (BBB)");
MODULE_VERSION("1.2");

#define DEVICE_NAME "led_driver"
#define CLASS_NAME "led_class"
#define BUFFER_SIZE 1024

#define GPIO1_BASE 0x4804C000
#define GPIO_SIZE  0x1000

#define GPIO_OE       0x134
#define GPIO_DATAIN   0x138
#define GPIO_DATAOUT  0x13C

// Định nghĩa các GPIO pin cho 3 LED
#define GPIO_LED1     117  // GPIO3_19 = P9_25
#define GPIO_LED2     18  // GPIO1_18 = GPIO số 50
#define GPIO_LED3     19  // GPIO1_19 = GPIO số 51

static int major_number;
static struct class *dev_class = NULL;
static struct device *dev_device = NULL;
static char kernel_buffer[BUFFER_SIZE];
static size_t buffer_size = 0;

static void __iomem *gpio_base = NULL;

static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char __user *, size_t, loff_t *);

static struct file_operations fops = {
    .open = device_open,
    .release = device_release,
    .read = device_read,
    .write = device_write,
};

// Hàm bật LED
static void led_set(int led_num, int state)
{
    int pin;
    int val = ioread32(gpio_base + GPIO_DATAOUT);
    
    // Xác định pin tương ứng với LED
    switch(led_num) {
        case 1:
            pin = GPIO_LED1;
            break;
        case 2:
            pin = GPIO_LED2;
            break;
        case 3:
            pin = GPIO_LED3;
            break;
        default:
            printk(KERN_WARNING "Invalid LED number: %d\n", led_num);
            return;
    }
    
    // Thiết lập trạng thái LED
    if (state)
        val |= (1 << pin);  // Bật LED
    else
        val &= ~(1 << pin); // Tắt LED
        
    iowrite32(val, gpio_base + GPIO_DATAOUT);
    printk(KERN_INFO "LED%d %s\n", led_num, state ? "ON" : "OFF");
}

// Hàm đọc trạng thái LED
static int led_get(int led_num)
{
    int pin;
    int val = ioread32(gpio_base + GPIO_DATAOUT);
    
    // Xác định pin tương ứng với LED
    switch(led_num) {
        case 1:
            pin = GPIO_LED1;
            break;
        case 2:
            pin = GPIO_LED2;
            break;
        case 3:
            pin = GPIO_LED3;
            break;
        default:
            return -1;
    }
    
    return (val & (1 << pin)) ? 1 : 0;
}

// Hàm khởi tạo
static int __init device_init(void)
{
    int val;

    printk(KERN_INFO "Initializing 3-LED GPIO driver\n");

    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ALERT "Failed to register major number\n");
        return major_number;
    }
    printk(KERN_INFO "Registered with major number: %d\n", major_number);

    dev_class = class_create(CLASS_NAME);
    if (IS_ERR(dev_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(dev_class);
    }
    printk(KERN_INFO "Class created successfully\n");

    dev_device = device_create(dev_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(dev_device)) {
        class_destroy(dev_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(dev_device);
    }
    printk(KERN_INFO "Device created successfully\n");

    memset(kernel_buffer, 0, BUFFER_SIZE);

    // Ánh xạ vùng nhớ GPIO1
    gpio_base = ioremap(GPIO1_BASE, GPIO_SIZE);
    if (!gpio_base) {
        printk(KERN_ALERT "Failed to ioremap GPIO\n");
        device_destroy(dev_class, MKDEV(major_number, 0));
        class_destroy(dev_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return -ENOMEM;
    }

    // Cấu hình GPIO1_17, GPIO1_18, GPIO1_19 là Output
    val = ioread32(gpio_base + GPIO_OE);
    val &= ~((1 << GPIO_LED1) | (1 << GPIO_LED2) | (1 << GPIO_LED3)); // Clear bits -> outputs
    iowrite32(val, gpio_base + GPIO_OE);
    
    // Khởi tạo tất cả LED ở trạng thái OFF
    val = ioread32(gpio_base + GPIO_DATAOUT);
    val &= ~((1 << GPIO_LED1) | (1 << GPIO_LED2) | (1 << GPIO_LED3)); // Tắt tất cả LED
    iowrite32(val, gpio_base + GPIO_DATAOUT);

    printk(KERN_INFO "Driver loaded. 3 LEDs configured as outputs and set to OFF.\n");
    return 0;
}

static void __exit device_exit(void)
{
    int val;
    
    // Tắt tất cả LED trước khi thoát
    if (gpio_base) {
        val = ioread32(gpio_base + GPIO_DATAOUT);
        val &= ~((1 << GPIO_LED1) | (1 << GPIO_LED2) | (1 << GPIO_LED3));
        iowrite32(val, gpio_base + GPIO_DATAOUT);
        iounmap(gpio_base);
    }
    
    device_destroy(dev_class, MKDEV(major_number, 0));
    class_destroy(dev_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_INFO "LED Driver unloaded\n");
}

static int device_open(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "Device opened\n");
    return 0;
}

static int device_release(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "Device closed\n");
    return 0;
}

// Đọc trạng thái tất cả LED
static ssize_t device_read(struct file *filep, char __user *buffer, size_t len, loff_t *offset)
{
    char result[16];
    int led1_state = led_get(1);
    int led2_state = led_get(2);
    int led3_state = led_get(3);
    int length;
    
    if (*offset > 0)
        return 0;
        
    // Chuẩn bị chuỗi kết quả chứa trạng thái 3 LED
    length = snprintf(result, sizeof(result), "%d%d%d\n", led1_state, led2_state, led3_state);
    
    if (copy_to_user(buffer, result, length) != 0)
        return -EFAULT;
        
    *offset += length;
    return length;
}

// Ghi trạng thái LED
static ssize_t device_write(struct file *filep, const char __user *buffer, size_t len, loff_t *offset)
{
    char cmd[16];
    int led_num = 0;
    int state = 0;
    
    if (len > sizeof(cmd) - 1)
        return -EINVAL;
        
    if (copy_from_user(cmd, buffer, len) != 0)
        return -EFAULT;
        
    cmd[len] = '\0';
    
    // Phân tích cú pháp đầu vào, định dạng: "LED:STATE"
    // Ví dụ: "1:1" - Bật LED 1, "2:0" - Tắt LED 2, "3:1" - Bật LED 3
    if (sscanf(cmd, "%d:%d", &led_num, &state) == 2) {
        if (led_num >= 1 && led_num <= 3 && (state == 0 || state == 1)) {
            led_set(led_num, state);
            return len;
        }
    }
    
    // Xử lý chế độ nháy LED tuần tự - khi nhận lệnh "blink"
    if (strncmp(cmd, "blink", 5) == 0) {
        int i;
        for (i = 1; i <= 3; i++) {
            // Bật LED
            led_set(i, 1);
            // Đợi một chút (trong kernel mode, chỉ nên dùng cho test, không khuyến khích trong thực tế)
            mdelay(500);
            // Tắt LED
            led_set(i, 0);
            mdelay(200);
        }
        return len;
    }
    
    printk(KERN_WARNING "Invalid command format. Use 'LED:STATE' or 'blink'\n");
    return -EINVAL;
}

module_init(device_init);
module_exit(device_exit);