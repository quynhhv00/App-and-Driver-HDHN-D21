/**
 * BH1750 I2C Driver for BeagleBone Black using bit-banging
 * 
 * This driver implements I2C protocol using bit-banging on GPIO pins
 * to communicate with the BH1750 light sensor.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>

#define DEVICE_NAME "bh1750"
#define CLASS_NAME "bh1750_class"

/* BH1750 I2C Address */
#define BH1750_ADDR 0x23  // ADDR pin is tied to GND

/* BH1750 Commands */
#define BH1750_POWER_DOWN      0x00
#define BH1750_POWER_ON        0x01
#define BH1750_RESET           0x07
#define BH1750_CONT_H_RES_MODE 0x10  // Continuously H-Resolution Mode
#define BH1750_CONT_H_RES_MODE2 0x11 // Continuously H-Resolution Mode 2
#define BH1750_CONT_L_RES_MODE 0x13  // Continuously L-Resolution Mode
#define BH1750_ONE_TIME_H_RES_MODE 0x20 // One time H-Resolution Mode
#define BH1750_ONE_TIME_H_RES_MODE2 0x21 // One time H-Resolution Mode 2
#define BH1750_ONE_TIME_L_RES_MODE 0x23  // One time L-Resolution Mode

/* BeagleBone Black GPIO Base Address */
#define GPIO1_BASE 0x4804C000
#define GPIO_SIZE  0x1000

/* GPIO Register offsets */
#define GPIO_OE        0x134
#define GPIO_DATAIN    0x138
#define GPIO_DATAOUT   0x13C
#define GPIO_SETDATAOUT 0x194
#define GPIO_CLEARDATAOUT 0x190

#define SCL_PIN 17  // P9_23 - GPIO1_17
#define SDA_PIN 16  // P9_15 - GPIO1_16

/* GPIO Bit Masks */
#define SCL_MASK (1 << SCL_PIN)
#define SDA_MASK (1 << SDA_PIN)

/* Module Parameters */
static int major;
static struct class *bh1750_class = NULL;
static struct device *bh1750_device = NULL;
static struct cdev bh1750_cdev;
static void __iomem *gpio_base;

/* I2C Bit-banging timing (in microseconds) */
#define I2C_DELAY 5

/* Refresh timer parameters */
#define DEFAULT_REFRESH_INTERVAL 1000  // 1 second in milliseconds
static struct timer_list refresh_timer;
static unsigned int refresh_interval = DEFAULT_REFRESH_INTERVAL;
static bool auto_refresh_enabled = true;
module_param(refresh_interval, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(refresh_interval, "Refresh interval in milliseconds (default: 1000)");
module_param(auto_refresh_enabled, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(auto_refresh_enabled, "Enable automatic refresh (default: true)");

/* Measurement mode */
static unsigned char measurement_mode = BH1750_CONT_H_RES_MODE;

/* Light sensor data structure */
struct bh1750_data {
    unsigned int lux_value;
    unsigned long last_update;
    struct mutex lock;
    bool sensor_initialized;
    bool continuous_mode;
};

static struct bh1750_data sensor_data;

/* I2C Bit-banging Functions */
static inline void set_scl(int value)
{
    if (value)
        iowrite32(SCL_MASK, gpio_base + GPIO_SETDATAOUT);
    else
        iowrite32(SCL_MASK, gpio_base + GPIO_CLEARDATAOUT);
    udelay(I2C_DELAY);
}

static inline void set_sda(int value)
{
    if (value)
        iowrite32(SDA_MASK, gpio_base + GPIO_SETDATAOUT);
    else
        iowrite32(SDA_MASK, gpio_base + GPIO_CLEARDATAOUT);
    udelay(I2C_DELAY);
}

static inline int read_sda(void)
{
    u32 val;
    u32 oe_val;
    
    /* Set SDA pin as input */
    oe_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(oe_val | SDA_MASK, gpio_base + GPIO_OE);
    
    /* Read SDA value */
    val = ioread32(gpio_base + GPIO_DATAIN);
    
    /* Set SDA pin as output again */
    iowrite32(oe_val, gpio_base + GPIO_OE);
    
    return (val & SDA_MASK) ? 1 : 0;
}

/* Set SDA as input or output */
static inline void sda_dir_in(void)
{
    u32 reg_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(reg_val | SDA_MASK, gpio_base + GPIO_OE);
    udelay(I2C_DELAY);
}

static inline void sda_dir_out(void)
{
    u32 reg_val = ioread32(gpio_base + GPIO_OE);
    iowrite32(reg_val & ~SDA_MASK, gpio_base + GPIO_OE);
    udelay(I2C_DELAY);
}

/* I2C Protocol Implementation */
static void i2c_start(void)
{
    sda_dir_out();
    set_sda(1);
    set_scl(1);
    set_sda(0);
    set_scl(0);
}

static void i2c_stop(void)
{
    sda_dir_out();
    set_sda(0);
    set_scl(1);
    set_sda(1);
}

static int i2c_write_byte(unsigned char byte)
{
    int i;
    int ack;
    
    sda_dir_out();
    
    /* Send 8 bits */
    for (i = 7; i >= 0; i--) {
        set_sda((byte >> i) & 1);
        set_scl(1);
        set_scl(0);
    }
    
    /* Read ACK */
    sda_dir_in();
    set_scl(1);
    ack = !read_sda();  // ACK = 0, NACK = 1
    set_scl(0);
    
    return ack;
}

static unsigned char i2c_read_byte(int ack)
{
    int i;
    unsigned char byte = 0;
    
    sda_dir_in();
    
    /* Read 8 bits */
    for (i = 7; i >= 0; i--) {
        set_scl(1);
        if (read_sda())
            byte |= (1 << i);
        set_scl(0);
    }
    
    /* Send ACK/NACK */
    sda_dir_out();
    set_sda(!ack);  // ACK = 0, NACK = 1
    set_scl(1);
    set_scl(0);
    
    return byte;
}

/* BH1750 Communication Functions */
static int bh1750_write_command(unsigned char cmd)
{
    int ret;
    
    i2c_start();
    ret = i2c_write_byte(BH1750_ADDR << 1);  // Write address
    if (!ret) {
        i2c_stop();
        printk(KERN_ERR "BH1750: No ACK on address write (addr=0x%02x)\n", BH1750_ADDR);
        return -EIO;
    }
    
    ret = i2c_write_byte(cmd);
    if (!ret) {
        i2c_stop();
        printk(KERN_ERR "BH1750: No ACK on command write (cmd=0x%02x)\n", cmd);
        return -EIO;
    }
    
    i2c_stop();
    return 0;
}

static int bh1750_read_value(unsigned short *value)
{
    unsigned char msb, lsb;
    int ret;
    
    i2c_start();
    ret = i2c_write_byte((BH1750_ADDR << 1) | 1);  // Read address
    if (!ret) {
        i2c_stop();
        printk(KERN_ERR "BH1750: No ACK on address read (addr=0x%02x)\n", BH1750_ADDR);
        return -EIO;
    }
    
    msb = i2c_read_byte(1);  // Read MSB with ACK
    lsb = i2c_read_byte(0);  // Read LSB with NACK
    
    i2c_stop();
    
    *value = (msb << 8) | lsb;
    return 0;
}

/* Get waiting time based on measurement mode */
static unsigned int get_measurement_wait_time(unsigned char mode)
{
    switch (mode) {
    case BH1750_CONT_H_RES_MODE:
    case BH1750_CONT_H_RES_MODE2:
    case BH1750_ONE_TIME_H_RES_MODE:
    case BH1750_ONE_TIME_H_RES_MODE2:
        return 180; // 180ms for high resolution modes
    case BH1750_CONT_L_RES_MODE:
    case BH1750_ONE_TIME_L_RES_MODE:
        return 24;  // 24ms for low resolution modes
    default:
        return 180; // Default to high resolution timing
    }
}

/* Check if mode is continuous */
static bool is_continuous_mode(unsigned char mode)
{
    return (mode == BH1750_CONT_H_RES_MODE || 
            mode == BH1750_CONT_H_RES_MODE2 || 
            mode == BH1750_CONT_L_RES_MODE);
}

static int bh1750_init_sensor(void)
{
    int ret;
    
    /* Power on the sensor */
    ret = bh1750_write_command(BH1750_POWER_ON);
    if (ret < 0)
        return ret;
        
    /* Reset data register */
    ret = bh1750_write_command(BH1750_RESET);
    if (ret < 0)
        return ret;
        
    /* Set measurement mode */
    ret = bh1750_write_command(measurement_mode);
    if (ret < 0)
        return ret;
        
    /* Wait for first measurement to be ready */
    msleep(get_measurement_wait_time(measurement_mode));
    
    sensor_data.sensor_initialized = true;
    sensor_data.continuous_mode = is_continuous_mode(measurement_mode);
    
    return 0;
}

static int bh1750_read_lux(unsigned int *lux)
{
    unsigned short raw_value;
    int ret;
    
    mutex_lock(&sensor_data.lock);
    
    /* If sensor is not initialized or not in continuous mode, we need to initialize/trigger a measurement */
    if (!sensor_data.sensor_initialized) {
        ret = bh1750_init_sensor();
        if (ret < 0) {
            mutex_unlock(&sensor_data.lock);
            return ret;
        }
    } else if (!sensor_data.continuous_mode) {
        /* For one-time measurement modes, we need to trigger a new measurement */
        ret = bh1750_write_command(measurement_mode);
        if (ret < 0) {
            mutex_unlock(&sensor_data.lock);
            return ret;
        }
        msleep(get_measurement_wait_time(measurement_mode));
    }
    
    /* Read data from sensor */
    ret = bh1750_read_value(&raw_value);
    if (ret < 0) {
        mutex_unlock(&sensor_data.lock);
        return ret;
    }
    
    /* Convert to lux */
    *lux = (raw_value * 10) / 12; // Adjusted per BH1750 datasheet: (raw_value / 1.2)
    
    /* Update last update timestamp */
    sensor_data.last_update = jiffies;
    sensor_data.lux_value = *lux;
    
    mutex_unlock(&sensor_data.lock);
    return 0;
}

/* Timer callback to refresh sensor data */
static void refresh_timer_callback(struct timer_list *t)
{
    unsigned int lux;
    int ret;
    
    /* Only refresh if auto-refresh is enabled */
    if (auto_refresh_enabled) {
        ret = bh1750_read_lux(&lux);
        if (ret < 0) {
            printk(KERN_ERR "BH1750: Failed to refresh sensor data: %d\n", ret);
        }
    }
    
    /* Reschedule timer */
    mod_timer(&refresh_timer, jiffies + msecs_to_jiffies(refresh_interval));
}

/* Set sensor measurement mode */
static int bh1750_set_mode(unsigned char mode)
{
    int ret;
    
    mutex_lock(&sensor_data.lock);
    
    /* Check if the mode is valid */
    switch (mode) {
    case BH1750_CONT_H_RES_MODE:
    case BH1750_CONT_H_RES_MODE2:
    case BH1750_CONT_L_RES_MODE:
    case BH1750_ONE_TIME_H_RES_MODE:
    case BH1750_ONE_TIME_H_RES_MODE2:
    case BH1750_ONE_TIME_L_RES_MODE:
        break;
    default:
        mutex_unlock(&sensor_data.lock);
        return -EINVAL;
    }
    
    /* Update the measurement mode */
    measurement_mode = mode;
    
    /* Send the command to the sensor */
    ret = bh1750_write_command(mode);
    if (ret < 0) {
        mutex_unlock(&sensor_data.lock);
        return ret;
    }
    
    /* Update continuous mode flag */
    sensor_data.continuous_mode = is_continuous_mode(mode);
    
    /* Wait for the measurement to be ready */
    msleep(get_measurement_wait_time(mode));
    
    mutex_unlock(&sensor_data.lock);
    return 0;
}

/* Driver File Operations */
static int bh1750_open(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "BH1750: Device opened\n");
    return 0;
}

static int bh1750_release(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "BH1750: Device closed\n");
    return 0;
}

static ssize_t bh1750_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    unsigned int lux;
    int ret;
    char lux_str[32];
    int len;
    
    if (*offset > 0)
        return 0;  // EOF
    
    /* Read lux value from sensor or use cached value if recent enough */
    if (time_after(jiffies, sensor_data.last_update + msecs_to_jiffies(refresh_interval)) || 
        !auto_refresh_enabled) {
        ret = bh1750_read_lux(&lux);
        if (ret < 0)
            return ret;
    } else {
        /* Use cached value */
        mutex_lock(&sensor_data.lock);
        lux = sensor_data.lux_value;
        mutex_unlock(&sensor_data.lock);
    }
    
    len = snprintf(lux_str, sizeof(lux_str), "%u lux\n", lux);
    
    /* Copy to user space */
    if (copy_to_user(buf, lux_str, len))
        return -EFAULT;
    
    *offset += len;
    return len;
}

static ssize_t bh1750_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
    char kbuf[64];
    unsigned long cmd;
    
    if (count >= sizeof(kbuf))
        return -EINVAL;
    
    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;
    
    kbuf[count] = '\0';
    
    if (kstrtoul(kbuf, 0, &cmd))
        return -EINVAL;
    
    /* Handle commands from user space */
    switch (cmd) {
    case 0:  // Power down
        mutex_lock(&sensor_data.lock);
        bh1750_write_command(BH1750_POWER_DOWN);
        sensor_data.sensor_initialized = false;
        mutex_unlock(&sensor_data.lock);
        break;
    case 1:  // Power on and reset
        mutex_lock(&sensor_data.lock);
        bh1750_write_command(BH1750_POWER_ON);
        bh1750_write_command(BH1750_RESET);
        mutex_unlock(&sensor_data.lock);
        break;
    case 2:  // High resolution mode (continuous)
        bh1750_set_mode(BH1750_CONT_H_RES_MODE);
        break;
    case 3:  // Low resolution mode (continuous)
        bh1750_set_mode(BH1750_CONT_L_RES_MODE);
        break;
    case 4:  // High resolution mode 2 (continuous)
        bh1750_set_mode(BH1750_CONT_H_RES_MODE2);
        break;
    case 5:  // High resolution mode (one time)
        bh1750_set_mode(BH1750_ONE_TIME_H_RES_MODE);
        break;
    case 6:  // Low resolution mode (one time)
        bh1750_set_mode(BH1750_ONE_TIME_L_RES_MODE);
        break;
    case 7:  // High resolution mode 2 (one time)
        bh1750_set_mode(BH1750_ONE_TIME_H_RES_MODE2);
        break;
    case 8:  // Enable auto refresh
        auto_refresh_enabled = true;
        break;
    case 9:  // Disable auto refresh
        auto_refresh_enabled = false;
        break;
    case 10: // Force refresh
        {
            unsigned int lux;
            bh1750_read_lux(&lux);
        }
        break;
    default:
        if (cmd >= 100 && cmd <= 10000) {
            /* Set refresh interval (100ms - 10000ms) */
            refresh_interval = cmd;
            mod_timer(&refresh_timer, jiffies + msecs_to_jiffies(refresh_interval));
            return count;
        }
        return -EINVAL;
    }
    
    return count;
}

/* sysfs interface */
static ssize_t show_lux(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int lux;
    int ret;
    
    ret = bh1750_read_lux(&lux);
    if (ret < 0)
        return ret;
    
    return sprintf(buf, "%u\n", lux);
}
static DEVICE_ATTR(lux, S_IRUGO, show_lux, NULL);

static ssize_t show_refresh_interval(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", refresh_interval);
}

static ssize_t store_refresh_interval(struct device *dev, struct device_attribute *attr, 
                                    const char *buf, size_t count)
{
    unsigned int val;
    
    if (kstrtouint(buf, 0, &val))
        return -EINVAL;
    
    if (val < 100 || val > 10000)
        return -EINVAL;
    
    refresh_interval = val;
    mod_timer(&refresh_timer, jiffies + msecs_to_jiffies(refresh_interval));
    
    return count;
}
static DEVICE_ATTR(refresh_interval, S_IRUGO | S_IWUSR, 
                  show_refresh_interval, store_refresh_interval);

static ssize_t show_auto_refresh(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", auto_refresh_enabled ? 1 : 0);
}

static ssize_t store_auto_refresh(struct device *dev, struct device_attribute *attr, 
                                 const char *buf, size_t count)
{
    unsigned int val;
    
    if (kstrtouint(buf, 0, &val))
        return -EINVAL;
    
    auto_refresh_enabled = (val != 0);
    
    return count;
}
static DEVICE_ATTR(auto_refresh, S_IRUGO | S_IWUSR, show_auto_refresh, store_auto_refresh);

static ssize_t show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    const char *mode_str;
    
    switch (measurement_mode) {
    case BH1750_CONT_H_RES_MODE:
        mode_str = "continuous high resolution";
        break;
    case BH1750_CONT_H_RES_MODE2:
        mode_str = "continuous high resolution 2";
        break;
    case BH1750_CONT_L_RES_MODE:
        mode_str = "continuous low resolution";
        break;
    case BH1750_ONE_TIME_H_RES_MODE:
        mode_str = "one time high resolution";
        break;
    case BH1750_ONE_TIME_H_RES_MODE2:
        mode_str = "one time high resolution 2";
        break;
    case BH1750_ONE_TIME_L_RES_MODE:
        mode_str = "one time low resolution";
        break;
    default:
        mode_str = "unknown";
    }
    
    return sprintf(buf, "%s\n", mode_str);
}

static ssize_t store_mode(struct device *dev, struct device_attribute *attr, 
                         const char *buf, size_t count)
{
    unsigned char mode;
    
    if (strncmp(buf, "cont-high", 9) == 0)
        mode = BH1750_CONT_H_RES_MODE;
    else if (strncmp(buf, "cont-high2", 10) == 0)
        mode = BH1750_CONT_H_RES_MODE2;
    else if (strncmp(buf, "cont-low", 8) == 0)
        mode = BH1750_CONT_L_RES_MODE;
    else if (strncmp(buf, "once-high", 9) == 0)
        mode = BH1750_ONE_TIME_H_RES_MODE;
    else if (strncmp(buf, "once-high2", 10) == 0)
        mode = BH1750_ONE_TIME_H_RES_MODE2;
    else if (strncmp(buf, "once-low", 8) == 0)
        mode = BH1750_ONE_TIME_L_RES_MODE;
    else
        return -EINVAL;
    
    if (bh1750_set_mode(mode) < 0)
        return -EIO;
    
    return count;
}
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode, store_mode);

static ssize_t store_refresh_now(struct device *dev, struct device_attribute *attr, 
                               const char *buf, size_t count)
{
    unsigned int val;
    unsigned int lux;
    
    if (kstrtouint(buf, 0, &val))
        return -EINVAL;
    
    if (val != 1)
        return -EINVAL;
    
    if (bh1750_read_lux(&lux) < 0)
        return -EIO;
    
    return count;
}
static DEVICE_ATTR(refresh_now, S_IWUSR, NULL, store_refresh_now);

/* File operations structure */
static const struct file_operations bh1750_fops = {
    .owner = THIS_MODULE,
    .open = bh1750_open,
    .release = bh1750_release,
    .read = bh1750_read,
    .write = bh1750_write,
};

/* GPIO Setup Function */
static int setup_gpio(void)
{
    u32 reg_val;
    
    /* Map GPIO registers */
    gpio_base = ioremap(GPIO1_BASE, GPIO_SIZE);
    if (!gpio_base) {
        printk(KERN_ERR "BH1750: Failed to map GPIO registers\n");
        return -ENOMEM;
    }
    
    /* Configure SCL and SDA pins as outputs */
    reg_val = ioread32(gpio_base + GPIO_OE);
    reg_val &= ~(SCL_MASK | SDA_MASK);  // Clear bits to set as output
    iowrite32(reg_val, gpio_base + GPIO_OE);
    
    /* Initialize SCL and SDA to high */
    iowrite32(SCL_MASK | SDA_MASK, gpio_base + GPIO_SETDATAOUT);
    
    return 0;
}

/* Create sysfs entries */
static int create_sysfs_entries(void)
{
    int ret;
    
    ret = device_create_file(bh1750_device, &dev_attr_lux);
    if (ret < 0)
        return ret;
    
    ret = device_create_file(bh1750_device, &dev_attr_refresh_interval);
    if (ret < 0)
        goto fail_refresh_interval;
    
    ret = device_create_file(bh1750_device, &dev_attr_auto_refresh);
    if (ret < 0)
        goto fail_auto_refresh;
    
    ret = device_create_file(bh1750_device, &dev_attr_mode);
    if (ret < 0)
        goto fail_mode;
    
    ret = device_create_file(bh1750_device, &dev_attr_refresh_now);
    if (ret < 0)
        goto fail_refresh_now;
    
    return 0;
    
fail_refresh_now:
    device_remove_file(bh1750_device, &dev_attr_mode);
fail_mode:
    device_remove_file(bh1750_device, &dev_attr_auto_refresh);
fail_auto_refresh:
    device_remove_file(bh1750_device, &dev_attr_refresh_interval);
fail_refresh_interval:
    device_remove_file(bh1750_device, &dev_attr_lux);
    return ret;
}

/* Remove sysfs entries */
static void remove_sysfs_entries(void)
{
    device_remove_file(bh1750_device, &dev_attr_refresh_now);
    device_remove_file(bh1750_device, &dev_attr_mode);
    device_remove_file(bh1750_device, &dev_attr_auto_refresh);
    device_remove_file(bh1750_device, &dev_attr_refresh_interval);
    device_remove_file(bh1750_device, &dev_attr_lux);
}

/* Module Initialization */
static int __init bh1750_init(void)
{
    int ret;
    dev_t dev = 0;
    
    printk(KERN_INFO "BH1750: Initializing driver\n");
    
    /* Initialize sensor data */
    mutex_init(&sensor_data.lock);
    sensor_data.lux_value = 0;
    sensor_data.last_update = 0;
    sensor_data.sensor_initialized = false;
    sensor_data.continuous_mode = true;
    
    /* Dynamically allocate major/minor numbers */
    ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to allocate device numbers\n");
        return ret;
    }
    major = MAJOR(dev);
    
    /* Initialize character device */
    cdev_init(&bh1750_cdev, &bh1750_fops);
    bh1750_cdev.owner = THIS_MODULE;
    ret = cdev_add(&bh1750_cdev, dev, 1);
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to add character device\n");
        goto fail_cdev;
    }
    
    /* Create device class */
    bh1750_class = class_create(CLASS_NAME);
    if (IS_ERR(bh1750_class)) {
        printk(KERN_ERR "BH1750: Failed to create device class\n");
        ret = PTR_ERR(bh1750_class);
        goto fail_class;
    }
    
    /* Create device file */
    bh1750_device = device_create(bh1750_class, NULL, dev, NULL, DEVICE_NAME);
    if (IS_ERR(bh1750_device)) {
        printk(KERN_ERR "BH1750: Failed to create device\n");
        ret = PTR_ERR(bh1750_device);
        goto fail_device;
    }
    
    /* Setup GPIO */
    ret = setup_gpio();
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to setup GPIO\n");
        goto fail_gpio;
    }
    
    /* Create sysfs entries */
    ret = create_sysfs_entries();
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to create sysfs entries\n");
        goto fail_sysfs;
    }
    
    /* Initialize timer */
    timer_setup(&refresh_timer, refresh_timer_callback, 0);
    mod_timer(&refresh_timer, jiffies + msecs_to_jiffies(refresh_interval));
    
    /* Initialize sensor */
    ret = bh1750_init_sensor();
    if (ret < 0) {
        printk(KERN_ERR "BH1750: Failed to initialize sensor\n");
        goto fail_sensor;
    }
    
    printk(KERN_INFO "BH1750: Driver initialized successfully\n");
    return 0;

fail_sensor:
    del_timer_sync(&refresh_timer);
    remove_sysfs_entries();
fail_sysfs:
    iounmap(gpio_base);
fail_gpio:
    device_destroy(bh1750_class, MKDEV(major, 0));
fail_device:
    class_destroy(bh1750_class);
fail_class:
    cdev_del(&bh1750_cdev);
fail_cdev:
    unregister_chrdev_region(MKDEV(major, 0), 1);
    return ret;
}

/* Module Cleanup */
static void __exit bh1750_exit(void)
{
    printk(KERN_INFO "BH1750: Removing driver\n");
    
    /* Stop timer */
    del_timer_sync(&refresh_timer);
    
    /* Power down the sensor */
    mutex_lock(&sensor_data.lock);
    bh1750_write_command(BH1750_POWER_DOWN);
    mutex_unlock(&sensor_data.lock);
    
    /* Remove sysfs entries */
    remove_sysfs_entries();
    
    /* Unmap GPIO registers */
    if (gpio_base)
        iounmap(gpio_base);
    
    /* Remove device and class */
    device_destroy(bh1750_class, MKDEV(major, 0));
    class_destroy(bh1750_class);
    
    /* Remove character device and release major/minor numbers */
    cdev_del(&bh1750_cdev);
    unregister_chrdev_region(MKDEV(major, 0), 1);
    
    printk(KERN_INFO "BH1750: Driver removed\n");
}

module_init(bh1750_init);
module_exit(bh1750_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bach");
MODULE_DESCRIPTION("BH1750 I2C Driver using GPIO bit-banging for BeagleBone Black");
MODULE_VERSION("1.0");