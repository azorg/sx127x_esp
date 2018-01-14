Connect Ra-01 module base on LoRaTM sx127x chip to ESP8266 under MicroPython
============================================================================

### Notes
1. This is experimental example, not software product ready for use
2. This this free and open source software
3. Author: Alex Zorg azorg(at)mail.ru
4. Licenced by GPLv3
5. Some sources based on:
 * https://github.com/Wei1234c/SX127x_driver_for_MicroPython_on_ESP8266
 * https://wei1234c.blogspot.tw/2017/08/sx127x-lora-transceiver-driver-for.html

## Download Micropython firmware
Go to:

1. "Getting started with MicroPython on the ESP8266"
http://docs.micropython.org/en/latest/esp8266/esp8266/tutorial/intro.html

2. "Firmware for ESP8266 boards"
http://micropython.org/download#esp8266

## Install `esptool` and flash firmware on ESP8266 bord
```
$ pip install esptool

$ esptool.py --port /dev/ttyUSB0 erase_flash
$ esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash --flash_size=detect 0 firmware.bin
```

## Connect LoRa module to ESP8266

| ESP GPIO | NodeMCU v3 |   Signal    | SX1278 (color)  |
| -------- | ---------- | ----------- | --------------- |
|     0    |     D3     |             |                 |
|     2    |     -      |  Blue LED   |                 |
|     4    |     D2     |    IRQ      | DIO0  (yellow)  |
|     5    |     D1     |    RESET    | RESET (magenta) |
|    10    |     SK     |             |                 |
|    12    |     D6     |    MISO     | MISO  (blue)    |
|    13    |     D7     |    MOSI     | MOSI  (green)   |
|    14    |     D5     |    SCK      | SCK   (white)   |
|    15    |     D8     |    CS       | NSS   (grey)    |
|    16    |     D0     |             |                 |
|          |     3V     |    3.3V     | 3.3V  (red)     |
|          |     G      |    GND      | GND   (black)   |

## Build mpy-cross

```
$ sudo apt-get install build-essential libreadline-dev libffi-dev git

$ git clone --recurse-submodules https://github.com/micropython/micropython.git

$ cd ./micropython/ports/unix
$ make axtls
$ make
$ sudo cp ./micropython /usr/local/bin

$ cd ../../mpy-cross
$ make
$ sudo cp ./mpy-cross /usr/local/bin
```

## Load python examples to module
```
$ mpy-cross -O3 lora.py
$ ampy --port /dev/ttyUSB0 put lora.mpy
$ ampy --port /dev/ttyUSB0 put main.py
```

## Run terminal

> $ picomom /dev/ttyUSB0 -b 115200


