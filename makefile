PROGRAM = temperature_sensor

EXTRA_COMPONENTS = \
	extras/dht \
	extras/http-parser \
	extras/dhcpserver \
	extras/i2c \
	extras/bmp280 \
	extras/pwm \
	$(abspath ../../components/esp-8266/wifi_config) \
	$(abspath ../../components/esp-8266/cJSON) \
	$(abspath ../../components/common/wolfssl) \
	$(abspath ../../components/common/homekit)


FLASH_SIZE ?= 32

EXTRA_CFLAGS += -I../.. -DHOMEKIT_SHORT_APPLE_UUIDS

include $(SDK_PATH)/common.mk

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud 115200 --elf $(PROGRAM_OUT)
