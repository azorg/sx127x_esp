#!/bin/sh

if true
then
  # ESP8266
  #FIRMWARE="esp8266-20171101-v1.9.3.bin"
  FIRMWARE="esp8266-20180205-v1.9.3-278-gcc92c057.bin"

  sudo esptool.py --port /dev/ttyUSB0 erase_flash
  sudo esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash \
                  --flash_size=detect 0 "$FIRMWARE"

else
  # ESP32
  FIRMWARE="esp32-20180204-v1.9.3-269-g253f2bd7.bin"

  sudo esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
  sudo esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash \
                  -z 0x1000 "$FIRMWARE"
fi

