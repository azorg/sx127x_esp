#!/bin/sh

#FIRMWARE="esp8266-20171101-v1.9.3.bin"
FIRMWARE="esp8266-20180205-v1.9.3-278-gcc92c057.bin"

sudo esptool.py --port /dev/ttyUSB0 erase_flash

sudo esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash \
	--flash_size=detect 0 "$FIRMWARE"

