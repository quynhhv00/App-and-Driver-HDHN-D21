#!/bin/sh
# Script to load BBB drivers and run app

case "$1" in
    start)
        echo "Loading BBB drivers and starting app..."
        insmod /lib/modules/bh1750_driverv2.ko
        insmod /lib/modules/led_driver.ko
        insmod /lib/modules/dht11_driver.ko
        /usr/bin/app &
        ;;
    stop)
        echo "Stopping app..."
        pid=$(pidof app)
        if [ -n "$pid" ]; then
            kill -9 $pid || true
        fi
        sleep 5
        rmmod -f bh1750_driverv2 || true
        rmmod -f led_driver || true
        rmmod -f dht11_driver || true
        ;;
    *)
        echo "Usage: $0 {start|stop}"
        exit 1
        ;;
esac

exit 0