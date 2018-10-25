deps_config := \
	/home/thor/esp/ESP8266_RTOS_SDK/components/aws_iot/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/esp8266/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/freertos/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/log/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/lwip/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/newlib/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/pthread/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/ssl/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/tcpip_adapter/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/wpa_supplicant/Kconfig \
	/home/thor/esp/ESP8266_RTOS_SDK/components/bootloader/Kconfig.projbuild \
	/home/thor/esp/ESP8266_RTOS_SDK/components/esptool_py/Kconfig.projbuild \
	/home/thor/esp/ESP8266_RTOS_SDK/examples/protocols/mqtt_thor/main/Kconfig.projbuild \
	/home/thor/esp/ESP8266_RTOS_SDK/components/partition_table/Kconfig.projbuild \
	/home/thor/esp/ESP8266_RTOS_SDK/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
