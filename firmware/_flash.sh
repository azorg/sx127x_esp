#!/bin/sh

sudo esptool.py --port /dev/ttyUSB0 erase_flash

sudo esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash \
	--flash_size=detect 0 esp8266-20171101-v1.9.3.bin

#sudo esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash \
#	--flash_size=detect 0 esp8266-20180112-v1.9.3-240-ga275cb0f.bin

