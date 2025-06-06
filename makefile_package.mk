BBB_SENSOR_VERSION = 1.0
BBB_SENSOR_SITE = $(TOPDIR)/package/bbb_sensor
BBB_SENSOR_SITE_METHOD = local
BBB_SENSOR_INSTALL_STAGING = YES
define BBB_SENSOR_BUILD_CMDS
   @echo "No build needed, using prebuilt files"
endef
define BBB_SENSOR_INSTALL_TARGET_CMDS
   # Copy app
   $(INSTALL) -D -m 0755 $(BBB_SENSOR_SITE)/app $(TARGET_DIR)/usr/bin/app
   # Copy driver modules
   $(INSTALL) -D -m 0644 $(BBB_SENSOR_SITE)/bh1750_driverv2.ko $(TARGET_DIR)/lib/modules/bh1750_driverv2.ko
   $(INSTALL) -D -m 0644 $(BBB_SENSOR_SITE)/led_driver.ko $(TARGET_DIR)/lib/modules/led_driver.ko
   $(INSTALL) -D -m 0644 $(BBB_SENSOR_SITE)/dht11_driver.ko $(TARGET_DIR)/lib/modules/dht11_driver.ko
   # Copy init script
   $(INSTALL) -D -m 0755 $(BBB_SENSOR_SITE)/S99bbb_drivers $(TARGET_DIR)/etc/init.d/S99bbb_drivers
endef
$(eval $(generic-package))
