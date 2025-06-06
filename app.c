#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <cjson/cJSON.h>
#include <mosquitto.h>
#include <linux/watchdog.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/resource.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>

#define DHT11_DEVICE_PATH "/dev/dht11_driver"
#define BH1750_DEVICE_PATH "/dev/bh1750"
#define LED_DEVICE_PATH "/dev/led_driver"
#define PWM_DEVICE_PATH "/dev/pwm_driver"
#define LOG_FILE "/var/log/sensor_system.log"
#define WATCHDOG_DEVICE "/dev/watchdog"
#define BUFFER_SIZE 128
#define MAX_RETRIES 3
#define MQTT_BROKER "192.168.137.1"
#define MQTT_PORT 1885
#define MQTT_CLIENT_ID "bbb_sensor_app"
#define MQTT_QOS 1
#define BLINK_INTERVAL 200000 /* 0.2 giây (micro giây) */
#define RECONNECT_INTERVAL 5 /* Giây */
#define WATCHDOG_TIMEOUT 120 /* Timeout watchdog */
#define MAX_LOG_SIZE (1024 * 1024) /* Giới hạn log 1MB */
#define LOG_STATUS_INTERVAL 300 /* Ghi trạng thái mỗi 5 phút */
#define IO_TIMEOUT_SECONDS 2 /* Timeout cho I/O */
#define DHT11_MAX_FAILS 5 /* Ngưỡng bỏ qua DHT11 */
#define DHT11_THREAD_TIMEOUT 10 /* Timeout thread DHT11 (giây) */

/* Biến toàn cục */
static int watchdog_fd = -1;
static struct mosquitto *mosq = NULL;
static int use_watchdog = 0; /* Tắt watchdog mặc định */
static volatile sig_atomic_t running = 1; /* Biến điều khiển vòng lặp */
static volatile sig_atomic_t dht11_enabled = 1; /* Trạng thái DHT11 */
static int dht11_fail_count = 0; /* Đếm lỗi DHT11 */
static volatile time_t last_loop_time = 0; /* Thời gian vòng lặp chính */
static volatile time_t last_dht11_time = 0; /* Thời gian đọc DHT11 */
static float last_temp = 0.0, last_humid = 0.0; /* Dữ liệu DHT11 mới nhất */
static pthread_t dht11_thread, monitor_thread;
static pthread_mutex_t dht11_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Khai báo prototype */
static void *dht11_thread_func(void *arg);
static void *monitor_thread_func(void *arg);
static int check_device(const char *path);
static int set_pwm_lux(unsigned int lux);

/* Hàm ghi log */
void log_data(const char *message) {
    struct stat log_stat;
    if (stat(LOG_FILE, &log_stat) == 0 && log_stat.st_size > MAX_LOG_SIZE) {
        rename(LOG_FILE, LOG_FILE ".bak");
    }
    FILE *log_fp = fopen(LOG_FILE, "a");
    if (log_fp) {
        time_t now = time(NULL);
        char *time_str = ctime(&now);
        time_str[strlen(time_str) - 1] = '\0';
        fprintf(log_fp, "[%s] %s\n", time_str, message);
        fclose(log_fp);
    }
}

/* Hàm ghi trạng thái hệ thống */
void log_system_status() {
    struct rusage usage;
    char buffer[BUFFER_SIZE];
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
        snprintf(buffer, sizeof(buffer), "System status: Memory usage: %ld KB, User time: %ld.%06ld s",
                 usage.ru_maxrss, usage.ru_utime.tv_sec, usage.ru_utime.tv_usec);
        log_data(buffer);
    }
    FILE *fp = popen("lsof -p $(pidof app) | wc -l", "r");
    if (fp) {
        int fd_count;
        if (fscanf(fp, "%d", &fd_count) == 1) {
            snprintf(buffer, sizeof(buffer), "System status: Open file descriptors: %d", fd_count);
            log_data(buffer);
        }
        pclose(fp);
    }
}

/* Hàm kiểm tra lỗi kernel */
void check_kernel_log() {
    char buffer[BUFFER_SIZE];
    FILE *fp = popen("dmesg | tail -n 10", "r");
    if (fp) {
        while (fgets(buffer, sizeof(buffer), fp) != NULL) {
            buffer[strcspn(buffer, "\n")] = '\0';
            char log_msg[BUFFER_SIZE];
            snprintf(log_msg, sizeof(log_msg), "Kernel log: %s", buffer);
            log_data(log_msg);
        }
        pclose(fp);
    }
}

/* Thread giám sát */
void *monitor_thread_func(void *arg) {
    while (running) {
        time_t now = time(NULL);
        if (now - last_loop_time > 5) {
            log_data("Monitor: Main loop appears to be stuck");
            check_kernel_log();
        }
        if (dht11_enabled && now - last_dht11_time > DHT11_THREAD_TIMEOUT) {
            log_data("Monitor: DHT11 thread appears to be stuck, restarting");
            pthread_cancel(dht11_thread);
            pthread_join(dht11_thread, NULL);
            if (pthread_create(&dht11_thread, NULL, dht11_thread_func, NULL) != 0) {
                log_data("Monitor: Failed to restart DHT11 thread");
            }
        }
        sleep(2);
    }
    return NULL;
}

/* Thread đọc DHT11 */
void *dht11_thread_func(void *arg) {
    float temp, humid;
    int fd = -1;
    char buffer[BUFFER_SIZE];
    ssize_t bytes_read;
    int retries;

    while (running && dht11_enabled) {
        retries = 0;
        log_data("DHT11: Attempting to read");
        if (check_device(DHT11_DEVICE_PATH) != 0) {
            log_data("DHT11: Device check failed");
            pthread_mutex_lock(&dht11_mutex);
            dht11_fail_count++;
            pthread_mutex_unlock(&dht11_mutex);
            sleep(1);
            continue;
        }

        while (retries < MAX_RETRIES && running && dht11_enabled) {
            fd = open(DHT11_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
            if (fd < 0) {
                fprintf(stderr, "DHT11: Failed to open device: %s\n", strerror(errno));
                log_data("DHT11: Failed to open device");
                pthread_mutex_lock(&dht11_mutex);
                dht11_fail_count++;
                pthread_mutex_unlock(&dht11_mutex);
                sleep(1);
                break;
            }

            struct timeval timeout;
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(fd, &read_fds);
            timeout.tv_sec = IO_TIMEOUT_SECONDS;
            timeout.tv_usec = 0;

            int ret = select(fd + 1, &read_fds, NULL, NULL, &timeout);
            if (ret <= 0) {
                fprintf(stderr, "DHT11: Select timeout or error: %s\n", ret == 0 ? "Timeout" : strerror(errno));
                log_data("DHT11: Select timeout or error");
                close(fd);
                retries++;
                pthread_mutex_lock(&dht11_mutex);
                dht11_fail_count++;
                pthread_mutex_unlock(&dht11_mutex);
                usleep(100000);
                continue;
            }

            memset(buffer, 0, sizeof(buffer));
            bytes_read = read(fd, buffer, sizeof(buffer) - 1);
            if (bytes_read < 0) {
                fprintf(stderr, "DHT11: Failed to read device: %s\n", strerror(errno));
                log_data("DHT11: Failed to read device");
                close(fd);
                retries++;
                pthread_mutex_lock(&dht11_mutex);
                dht11_fail_count++;
                pthread_mutex_unlock(&dht11_mutex);
                usleep(100000);
                continue;
            }

            buffer[bytes_read] = '\0';
            char raw_data[BUFFER_SIZE];
            snprintf(raw_data, sizeof(raw_data), "DHT11: Raw data: '%s'", buffer);
            log_data(raw_data);

            int temp_int, humid_int;
            if (sscanf(buffer, "Temp: %dC, Hum: %d%%", &temp_int, &humid_int) != 2) {
                fprintf(stderr, "DHT11: Failed to parse data: '%s' (bytes read: %zd)\n", buffer, bytes_read);
                snprintf(buffer, sizeof(buffer), "DHT11: Failed to parse data: '%s'", buffer);
                log_data(buffer);
                close(fd);
                retries++;
                pthread_mutex_lock(&dht11_mutex);
                dht11_fail_count++;
                pthread_mutex_unlock(&dht11_mutex);
                usleep(100000);
                continue;
            }

            temp = (float)temp_int;
            humid = (float)humid_int;
            close(fd);
            log_data("DHT11: Read successful");
            pthread_mutex_lock(&dht11_mutex);
            last_temp = temp;
            last_humid = humid;
            dht11_fail_count = 0;
            last_dht11_time = time(NULL);
            pthread_mutex_unlock(&dht11_mutex);
            break;
        }

        if (retries >= MAX_RETRIES) {
            log_data("DHT11: Failed to read after retries");
            pthread_mutex_lock(&dht11_mutex);
            dht11_fail_count++;
            pthread_mutex_unlock(&dht11_mutex);
        }

        if (dht11_fail_count >= DHT11_MAX_FAILS) {
            log_data("DHT11: Disabled due to excessive failures");
            check_kernel_log();
            dht11_enabled = 0;
        }

        sleep(3); /* Đọc mỗi 3 giây */
    }
    if (fd >= 0) close(fd);
    return NULL;
}

/* Hàm khởi tạo watchdog */
int init_watchdog(int *watchdog_fd) {
    if (!use_watchdog) {
        log_data("Watchdog disabled");
        return 0;
    }
    *watchdog_fd = open(WATCHDOG_DEVICE, O_RDWR);
    if (*watchdog_fd < 0) {
        fprintf(stderr, "Failed to open watchdog device: %s\n", strerror(errno));
        log_data("Failed to open watchdog device");
        return -1;
    }
    int timeout = WATCHDOG_TIMEOUT;
    if (ioctl(*watchdog_fd, WDIOC_SETTIMEOUT, &timeout) < 0) {
        fprintf(stderr, "Failed to set watchdog timeout: %s\n", strerror(errno));
        log_data("Failed to set watchdog timeout");
        close(*watchdog_fd);
        return -1;
    }
    if (ioctl(*watchdog_fd, WDIOC_KEEPALIVE, 0) < 0) {
        fprintf(stderr, "Failed to send initial watchdog keepalive: %s\n", strerror(errno));
        log_data("Failed to send initial watchdog keepalive");
        close(*watchdog_fd);
        return -1;
    }
    printf("Watchdog initialized with timeout %d seconds\n", timeout);
    log_data("Watchdog initialized");
    return 0;
}

/* Hàm gửi tín hiệu keepalive cho watchdog */
int ping_watchdog(int watchdog_fd) {
    if (!use_watchdog || watchdog_fd < 0) return 0;
    if (ioctl(watchdog_fd, WDIOC_KEEPALIVE, 0) < 0) {
        fprintf(stderr, "Failed to ping watchdog: %s\n", strerror(errno));
        log_data("Failed to ping watchdog");
        return -1;
    }
    return 0;
}

/* Hàm vô hiệu hóa watchdog */
void disable_watchdog(int watchdog_fd) {
    if (!use_watchdog || watchdog_fd < 0) return;
    if (write(watchdog_fd, "V", 1) < 0) {
        fprintf(stderr, "Failed to disable watchdog: %s\n", strerror(errno));
        log_data("Failed to disable watchdog");
    }
    close(watchdog_fd);
    log_data("Watchdog disabled");
}

/* Hàm kiểm tra trạng thái thiết bị */
int check_device(const char *path) {
    struct stat statbuf;
    if (stat(path, &statbuf) != 0) {
        char buffer[BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "Device %s not found: %s", path, strerror(errno));
        log_data(buffer);
        return -1;
    }
    return 0;
}

/* Hàm ghi giá trị lux vào PWM driver */
int set_pwm_lux(unsigned int lux) {
    int fd = -1;
    char buffer[16];

    log_data("PWM: Attempting to set lux");
    if (check_device(PWM_DEVICE_PATH) != 0) {
        log_data("PWM: Device check failed");
        return -1;
    }

    fd = open(PWM_DEVICE_PATH, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "PWM: Failed to open device: %s\n", strerror(errno));
        log_data("PWM: Failed to open device");
        return -1;
    }

    snprintf(buffer, sizeof(buffer), "%u", lux);
    if (write(fd, buffer, strlen(buffer)) < 0) {
        fprintf(stderr, "PWM: Failed to set lux %u: %s\n", lux, strerror(errno));
        snprintf(buffer, sizeof(buffer), "PWM: Failed to set lux %u", lux);
        log_data(buffer);
        close(fd);
        return -1;
    }

    close(fd);
    snprintf(buffer, sizeof(buffer), "PWM: Set lux %u successful", lux);
    log_data(buffer);
    return 0;
}

/* Hàm đọc dữ liệu ánh sáng từ BH1750 với timeout */
int read_bh1750(unsigned int *lux) {
    int fd = -1;
    char buffer[BUFFER_SIZE];
    ssize_t ret;
    int retries = 0;

    log_data("BH1750: Attempting to read");
    if (check_device(BH1750_DEVICE_PATH) != 0) {
        log_data("BH1750: Device check failed");
        return -1;
    }

    while (retries < MAX_RETRIES) {
        fd = open(BH1750_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            fprintf(stderr, "BH1750: Failed to open device: %s\n", strerror(errno));
            log_data("BH1750: Failed to open device");
            return -1;
        }

        struct timeval timeout;
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        timeout.tv_sec = IO_TIMEOUT_SECONDS;
        timeout.tv_usec = 0;

        int sel_ret = select(fd + 1, &read_fds, NULL, NULL, &timeout);
        if (sel_ret <= 0) {
            fprintf(stderr, "BH1750: Select timeout or error: %s\n", sel_ret == 0 ? "Timeout" : strerror(errno));
            log_data("BH1750: Select timeout or error");
            close(fd);
            retries++;
            usleep(100000);
            continue;
        }

        ret = read(fd, buffer, sizeof(buffer) - 1);
        if (ret < 0) {
            fprintf(stderr, "BH1750: Failed to read device: %s\n", strerror(errno));
            log_data("BH1750: Failed to read device");
            close(fd);
            retries++;
            usleep(100000);
            continue;
        }

        buffer[ret] = '\0';
        char raw_data[BUFFER_SIZE];
        snprintf(raw_data, sizeof(raw_data), "BH1750: Raw data: '%s'", buffer);
        log_data(raw_data);

        if (sscanf(buffer, "%u", lux) != 1) {
            fprintf(stderr, "BH1750: Failed to parse lux value: '%s' (bytes read: %zd)\n", buffer, ret);
            snprintf(buffer, sizeof(buffer), "BH1750: Failed to parse lux value: '%s'", buffer);
            log_data(buffer);
            close(fd);
            retries++;
            usleep(100000);
            continue;
        }

        close(fd);
        log_data("BH1750: Read successful");
        return 0;
    }

    fprintf(stderr, "BH1750: Failed to read after %d retries\n", MAX_RETRIES);
    log_data("BH1750: Failed to read after retries");
    if (fd >= 0) close(fd);
    return -1;
}

/* Hàm điều khiển LED */
int set_led_state(int led_num, int state) {
    int fd = -1;
    char buffer[16];

    log_data("LED: Attempting to set state");
    if (check_device(LED_DEVICE_PATH) != 0) {
        log_data("LED: Device check failed");
        return -1;
    }

    fd = open(LED_DEVICE_PATH, O_WRONLY);
    if (fd < 0) {
        fprintf(stderr, "LED: Failed to open device: %s\n", strerror(errno));
        log_data("LED: Failed to open device");
        return -1;
    }

    sprintf(buffer, "%d:%d", led_num, state);
    if (write(fd, buffer, strlen(buffer)) < 0) {
        fprintf(stderr, "LED: Failed to control LED %d: %s\n", led_num, strerror(errno));
        snprintf(buffer, sizeof(buffer), "LED: Failed to control LED %d", led_num);
        log_data(buffer);
        close(fd);
        return -1;
    }

    close(fd);
    log_data("LED: Set state successful");
    return 0;
}

/* Hàm nháy LED */
int blink_led(struct mosquitto *mosq, int led_num) {
    char buffer[BUFFER_SIZE];
    if (set_led_state(led_num, 1) == 0) {
        snprintf(buffer, sizeof(buffer), "LED %d blinking ON", led_num);
        log_data(buffer);
    } else {
        return -1;
    }
    usleep(BLINK_INTERVAL);
    if (set_led_state(led_num, 0) == 0) {
        snprintf(buffer, sizeof(buffer), "LED %d blinking OFF", led_num);
        log_data(buffer);
    } else {
        return -1;
    }
    usleep(BLINK_INTERVAL);
    return 0;
}

/* Hàm gửi trạng thái LED qua MQTT */
int publish_led_status(struct mosquitto *mosq, int led_num, const char *state) {
    char buffer[BUFFER_SIZE];
    log_data("MQTT: Attempting to publish LED status");
    cJSON *jobj = cJSON_CreateObject();
    if (!jobj) {
        log_data("MQTT: Failed to create cJSON object for LED status");
        return -1;
    }

    if (strcmp(state, "blinking") == 0) {
        cJSON_AddStringToObject(jobj, "state", "blinking");
    } else {
        cJSON_AddNumberToObject(jobj, "state", atoi(state));
    }

    char *payload = cJSON_PrintUnformatted(jobj);
    if (!payload) {
        log_data("MQTT: Failed to print cJSON object for LED status");
        cJSON_Delete(jobj);
        return -1;
    }

    char topic[32];
    snprintf(topic, sizeof(topic), "status/led/%d", led_num);
    int ret = mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, MQTT_QOS, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT: Failed to publish LED %d status to %s: %s\n", led_num, topic, mosquitto_strerror(ret));
        snprintf(buffer, sizeof(buffer), "MQTT: Failed to publish LED %d status to %s: %s", led_num, topic, mosquitto_strerror(ret));
        log_data(buffer);
        free(payload);
        cJSON_Delete(jobj);
        return -1;
    }

    printf("Published LED %d status to %s: %s\n", led_num, topic, payload);
    snprintf(buffer, sizeof(buffer), "MQTT: Published LED %d status to %s: %s", led_num, topic, payload);
    log_data(buffer);
    free(payload);
    cJSON_Delete(jobj);
    return 0;
}

/* Hàm callback cho log Mosquitto */
void mosquitto_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str) {
    char buffer[BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "MQTT: Mosquitto log: [Level %d] %s", level, str);
    log_data(buffer);
}

/* Hàm callback cho message MQTT */
void mosquitto_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    char buffer[BUFFER_SIZE];
    log_data("MQTT: Received message");
    cJSON *json = cJSON_Parse(message->payload);
    if (!json) {
        fprintf(stderr, "MQTT: Failed to parse message on %s: %s\n", message->topic, cJSON_GetErrorPtr());
        snprintf(buffer, sizeof(buffer), "MQTT: Failed to parse message on %s", message->topic);
        log_data(buffer);
        return;
    }

    cJSON *state = cJSON_GetObjectItem(json, "state");
    if (!cJSON_IsNumber(state)) {
        fprintf(stderr, "MQTT: Invalid state in message on %s\n", message->topic);
        snprintf(buffer, sizeof(buffer), "MQTT: Invalid state in message on %s", message->topic);
        log_data(buffer);
        cJSON_Delete(json);
        return;
    }

    int led_num = -1;
    if (strcmp(message->topic, "control/led/2") == 0) {
        led_num = 2;
    } else if (strcmp(message->topic, "control/led/3") == 0) {
        led_num = 3;
    }

    if (led_num != -1) {
        int state_value = state->valueint;
        if (set_led_state(led_num, state_value) == 0) {
            printf("LED %d set to %s\n", led_num, state_value ? "ON" : "OFF");
            snprintf(buffer, sizeof(buffer), "LED %d set to %s", led_num, state_value ? "ON" : "OFF");
            log_data(buffer);
            snprintf(buffer, sizeof(buffer), "%d", state_value);
            publish_led_status(mosq, led_num, buffer);
        }
    }

    cJSON_Delete(json);
}

/* Hàm gửi dữ liệu qua MQTT */
int publish_mqtt(struct mosquitto *mosq, const char *topic, const char *payload) {
    log_data("MQTT: Attempting to publish data");
    int ret = mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, MQTT_QOS, false);
    if (ret != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT: Failed to publish to %s: %s (error code: %d)\n", topic, mosquitto_strerror(ret), ret);
        char buffer[BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "MQTT: Failed to publish to %s: %s (error code: %d)", topic, mosquitto_strerror(ret), ret);
        log_data(buffer);
        return -1;
    }

    printf("Published to %s: %s\n", topic, payload);
    char buffer[BUFFER_SIZE];
    snprintf(buffer, sizeof(buffer), "MQTT: Published to %s: %s", topic, payload);
    log_data(buffer);
    return 0;
}

/* Hàm khởi tạo và kết nối MQTT */
int mqtt_init_connect(struct mosquitto **mosq) {
    char buffer[BUFFER_SIZE];
    log_data("MQTT: Initializing");
    mosquitto_lib_init();
    *mosq = mosquitto_new(MQTT_CLIENT_ID, true, NULL);
    if (!*mosq) {
        fprintf(stderr, "MQTT: Failed to create client\n");
        log_data("MQTT: Failed to create client");
        return -1;
    }

    mosquitto_log_callback_set(*mosq, mosquitto_log_callback);
    mosquitto_message_callback_set(*mosq, mosquitto_message_callback);

    int ret = mosquitto_connect(*mosq, MQTT_BROKER, MQTT_PORT, 120);
    if (ret != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT: Failed to connect to broker %s:%d: %s\n", MQTT_BROKER, MQTT_PORT, mosquitto_strerror(ret));
        snprintf(buffer, sizeof(buffer), "MQTT: Failed to connect to broker %s:%d: %s", MQTT_BROKER, MQTT_PORT, mosquitto_strerror(ret));
        log_data(buffer);
        mosquitto_destroy(*mosq);
        *mosq = NULL;
        return -1;
    }

    ret = mosquitto_subscribe(*mosq, NULL, "control/led/2", MQTT_QOS);
    if (ret != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT: Failed to subscribe to control/led/2: %s\n", mosquitto_strerror(ret));
        snprintf(buffer, sizeof(buffer), "MQTT: Failed to subscribe to control/led/2: %s", mosquitto_strerror(ret));
        log_data(buffer);
    }
    ret = mosquitto_subscribe(*mosq, NULL, "control/led/3", MQTT_QOS);
    if (ret != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT: Failed to subscribe to control/led/3: %s\n", mosquitto_strerror(ret));
        snprintf(buffer, sizeof(buffer), "MQTT: Failed to subscribe to control/led/3: %s", mosquitto_strerror(ret));
        log_data(buffer);
    }

    ret = mosquitto_loop_start(*mosq);
    if (ret != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT: Failed to start loop: %s\n", mosquitto_strerror(ret));
        snprintf(buffer, sizeof(buffer), "MQTT: Failed to start loop: %s", mosquitto_strerror(ret));
        log_data(buffer);
        mosquitto_destroy(*mosq);
        *mosq = NULL;
        return -1;
    }

    printf("MQTT: Connected to broker\n");
    log_data("MQTT: Connected to broker");
    return 0;
}

/* Hàm tái kết nối MQTT */
int mqtt_reconnect(struct mosquitto **mosq) {
    char buffer[BUFFER_SIZE];
    log_data("MQTT: Attempting to reconnect");
    if (*mosq) {
        mosquitto_loop_stop(*mosq, true);
        mosquitto_disconnect(*mosq);
        mosquitto_destroy(*mosq);
        *mosq = NULL;
    }

    int ret = mqtt_init_connect(mosq);
    if (ret != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT: Failed to reconnect\n");
        log_data("MQTT: Failed to reconnect");
        return -1;
    }
    return 0;
}

/* Hàm xử lý tín hiệu */
void signal_handler(int sig, siginfo_t *info, void *context) {
    printf("Received signal %d, shutting down...\n", sig);
    log_data("Received signal, shutting down");
    running = 0;
}

/* Hàm chính */
int main(int argc, char *argv[]) {
    unsigned int lux = 0;
    char log_buffer[BUFFER_SIZE];
    int led2_state = 0, led3_state = 0;
    int led2_blinking = 0, led3_blinking = 0;
    int mqtt_connected = 0;
    time_t last_status_log = 0;
    float temp, humid;

    /* Kiểm tra tham số dòng lệnh */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--watchdog") == 0) {
            use_watchdog = 1;
            printf("Watchdog enabled via command line\n");
            log_data("Watchdog enabled via command line");
        }
    }

    /* Cài đặt xử lý tín hiệu */
    struct sigaction sa;
    sa.sa_sigaction = signal_handler;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* Khởi tạo thread giám sát */
    if (pthread_create(&monitor_thread, NULL, monitor_thread_func, NULL) != 0) {
        fprintf(stderr, "Failed to create monitor thread: %s\n", strerror(errno));
        log_data("Failed to create monitor thread");
        return -1;
    }

    /* Khởi tạo thread DHT11 */
    if (pthread_create(&dht11_thread, NULL, dht11_thread_func, NULL) != 0) {
        fprintf(stderr, "Failed to create DHT11 thread: %s\n", strerror(errno));
        log_data("Failed to create DHT11 thread");
        return -1;
    }

    /* Khởi tạo watchdog */
    if (init_watchdog(&watchdog_fd) != 0) {
        fprintf(stderr, "Failed to initialize watchdog, continuing without watchdog\n");
        log_data("Failed to initialize watchdog, continuing without watchdog");
        use_watchdog = 0;
    }

    /* Khởi tạo và kết nối MQTT */
    if (mqtt_init_connect(&mosq) == 0) {
        mqtt_connected = 1;
    }

    printf("Starting sensor system...\n");
    log_data("Starting sensor system");

    while (running) {
        last_loop_time = time(NULL);
        ping_watchdog(watchdog_fd);

        /* Ghi trạng thái hệ thống định kỳ */
        time_t now = time(NULL);
        if (now - last_status_log >= LOG_STATUS_INTERVAL) {
            log_system_status();
            last_status_log = now;
        }

        /* Kiểm tra và tái kết nối MQTT nếu cần */
        if (!mqtt_connected || (mosq && mosquitto_socket(mosq) == -1)) {
            mqtt_connected = 0;
            printf("MQTT: Connection lost, attempting to reconnect...\n");
            log_data("MQTT: Connection lost, attempting to reconnect");
            if (mqtt_reconnect(&mosq) == 0) {
                mqtt_connected = 1;
            } else {
                sleep(RECONNECT_INTERVAL);
                continue;
            }
        }

        /* Lấy dữ liệu DHT11 từ thread */
        pthread_mutex_lock(&dht11_mutex);
        temp = last_temp;
        humid = last_humid;
        pthread_mutex_unlock(&dht11_mutex);

        if (dht11_enabled) {
            printf("DHT11 - Temperature: %.1f°C, Humidity: %.1f%%\n", temp, humid);
            snprintf(log_buffer, sizeof(log_buffer), "DHT11: Temperature: %.1f°C, Humidity: %.1f%%", temp, humid);
            log_data(log_buffer);

            if (mqtt_connected) {
                cJSON *jobj = cJSON_CreateObject();
                if (jobj) {
                    cJSON_AddNumberToObject(jobj, "temperature", temp);
                    char *temp_payload = cJSON_PrintUnformatted(jobj);
                    if (temp_payload) {
                        publish_mqtt(mosq, "sensor/dht11/temperature", temp_payload);
                        free(temp_payload);
                    }
                    cJSON_Delete(jobj);
                }

                jobj = cJSON_CreateObject();
                if (jobj) {
                    cJSON_AddNumberToObject(jobj, "humidity", humid);
                    char *humid_payload = cJSON_PrintUnformatted(jobj);
                    if (humid_payload) {
                        publish_mqtt(mosq, "sensor/dht11/humidity", humid_payload);
                        free(humid_payload);
                    }
                    cJSON_Delete(jobj);
                }
            }
        } else {
            printf("DHT11: Disabled, skipping\n");
            log_data("DHT11: Disabled, skipping");
        }

        /* Đọc dữ liệu từ BH1750 và điều khiển PWM */
        if (read_bh1750(&lux) == 0) {
            printf("BH1750 - Light intensity: %u lux\n", lux);
            snprintf(log_buffer, sizeof(log_buffer), "BH1750: Light intensity: %u lux", lux);
            log_data(log_buffer);

            /* Ghi giá trị lux vào PWM driver */
            if (set_pwm_lux(lux) == 0) {
                printf("PWM: Set lux %u\n", lux);
            } else {
                printf("PWM: Failed to set lux %u\n", lux);
            }

            if (mqtt_connected) {
                cJSON *jobj = cJSON_CreateObject();
                if (jobj) {
                    cJSON_AddNumberToObject(jobj, "light", lux);
                    char *lux_payload = cJSON_PrintUnformatted(jobj);
                    if (lux_payload) {
                        publish_mqtt(mosq, "sensor/bh1750/light", lux_payload);
                        free(lux_payload);
                    }
                    cJSON_Delete(jobj);
                }
            }
        } else {
            printf("BH1750: Failed to read data, retrying next cycle\n");
            log_data("BH1750: Failed to read data, retrying next cycle");
        }

        /* Kiểm tra điều kiện nháy LED */
        if (temp > 35.0) {
            if (!led2_blinking) {
                led2_blinking = 1;
                printf("LED 2 blinking (Temperature > 35°C)\n");
                log_data("LED 2 blinking (Temperature > 35°C)");
                if (mqtt_connected) {
                    publish_led_status(mosq, 2, "blinking");
                }
            }
            blink_led(mosq, 2);
        } else {
            if (led2_blinking) {
                led2_blinking = 0;
                printf("LED 2 stop blinking (Temperature <= 35°C)\n");
                log_data("LED 2 stop blinking (Temperature <= 35°C)");
                set_led_state(2, led2_state);
                if (mqtt_connected) {
                    char state_str[2];
                    snprintf(state_str, sizeof(state_str), "%d", led2_state);
                    publish_led_status(mosq, 2, state_str);
                }
            }
        }

        if (lux > 100) {
            if (!led3_blinking) {
                led3_blinking = 1;
                printf("LED 3 blinking (Light > 100 lux)\n");
                log_data("LED 3 blinking (Light > 100 lux)");
                if (mqtt_connected) {
                    publish_led_status(mosq, 3, "blinking");
                }
            }
            blink_led(mosq, 3);
        } else {
            if (led3_blinking) {
                led3_blinking = 0;
                printf("LED 3 stop blinking (Light <= 100 lux)\n");
                log_data("LED 3 stop blinking (Light <= 100 lux)");
                set_led_state(3, led3_state);
                if (mqtt_connected) {
                    char state_str[2];
                    snprintf(state_str, sizeof(state_str), "%d", led3_state);
                    publish_led_status(mosq, 3, state_str);
                }
            }
        }

        /* Kiểm tra vòng lặp Mosquitto */
        if (mosq) {
            log_data("MQTT: Running mosquitto loop");
            int ret = mosquitto_loop(mosq, 0, 1);
            if (ret != MOSQ_ERR_SUCCESS) {
                fprintf(stderr, "MQTT: Loop error: %s\n", mosquitto_strerror(ret));
                snprintf(log_buffer, sizeof(log_buffer), "MQTT: Loop error: %s", mosquitto_strerror(ret));
                log_data(log_buffer);
                mqtt_connected = 0;
            }
        }

        /* Đợi 5 giây trước khi đọc lại */
        sleep(5);
    }

    /* Dọn dẹp trước khi thoát */
    log_data("Cleaning up before exit");
    running = 0;
    pthread_cancel(dht11_thread);
    pthread_cancel(monitor_thread);
    pthread_join(dht11_thread, NULL);
    pthread_join(monitor_thread, NULL);
    pthread_mutex_destroy(&dht11_mutex);
    disable_watchdog(watchdog_fd);
    if (mosq) {
        mosquitto_loop_stop(mosq, true);
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
    }
    log_data("Application terminated gracefully");
    return 0;
}
