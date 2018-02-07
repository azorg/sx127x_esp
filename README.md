Connect Ra-01 module base on LoRaTM SX127x chip to ESP8266/ESP32 under MicroPython
==================================================================================

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

2. "Firmware for ESP8266/ESP32 boards"
http://micropython.org/download

## Install `esptool.py` - ESP8266 & ESP32 ROM Bootloader Utility
```
$ sudo apt-get install python-pip
$ sudo pip install esptool
```
OR
```
$ sudo apt-get install python-serial
$ sudo apt-get install python-ecdsa ecdsautils
$ sudo apt-get install python-slowaes
$ git clone https://github.com/espressif/esptool.git
$ cd esptool
$ sudo python setup.py install
```

## Flash firmware on ESP8266/ESP32 board
ESP8266:
```
$ esptool.py --port /dev/ttyUSB0 erase_flash
$ esptool.py --port /dev/ttyUSB0 --baud 115200 write_flash --flash_size=detect 0 firmware.bin
```
ESP32:
```
$ esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
$ esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z 0x1000 firmware.bin
```

## Connect LoRa module to ESP8266/ESP32

|   GPIO   | DOIT ESP32 | NodeMCU v3 |   Signal    | SX1278 (color)  |
| -------- | ---------- | ---------- | ----------- | --------------- |
|     0    |    -       |     D3     |             |                 |
|     2    |    D2      |     D4     |  Blue LED   |                 |
|     4    |    D4      |     D2     |    IRQ      | DIO0  (yellow)  |
|     5    |    D5      |     D1     | HARD RESET  | RESET (magenta) |
|     9    |    -       |     S2?    |             |                 |
|    10    |    -       |   S3/SK?   |             |                 |
|    12    |    D12     |     D6     |    MISO     | MISO  (blue)    |
|    13    |    D13     |     D7     |    MOSI     | MOSI  (green)   |
|    14    |    D14     |     D5     |    SCK      | SCK   (white)   |
|    15    |    D15     |     D8     |    CS       | NSS   (grey)    |
|    16    |    -       |     D0     |    DATA*    | DIO2* (brown)   |
|          |            |            |    DCLK*    | DIO1* (orange)  |
|          |    3V3     |     3V     |    3.3V     | 3.3V  (red)     |
|          |    GND     |     G      |    GND      | GND   (black)   |

Note: DIO2(DATA) is optional and may used in continuous FSK/OOK mode

## Build `mpy-cross`

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

## Install `ampy` - Utility to interact with a MicroPython board over a serial connection
```
$ sudo pip install adafruit-ampy
```
OR
```
$ sudo apt-get install python-click
$ git clone https://github.com/adafruit/ampy.git
$ cd ampy
$ sudo python setup.py install
```

## Read and edit "main.py" and "sx127x.py", compile "sx127x.py"
```
$ vim sx127x.py
$ vim main.py
$ mpy-cross -O3 sx127x.py
```

## Load python examples to ESP module
```
$ ampy --port /dev/ttyUSB0 put sx127x.mpy
$ ampy --port /dev/ttyUSB0 put main.py
```

## Run terminal (minicom, picocom or screen)
```
$ minicom -D /dev/ttyUSB0 -b 115200

$ picocom /dev/ttyUSB0 -b 115200

$ screen /dev/ttyUSB0 115200
```

