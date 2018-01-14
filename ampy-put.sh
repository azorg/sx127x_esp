#!/bin/sh


if mpy-cross -O3 main.py && mpy-cross -O3 lora.py
then
  sudo ampy --port /dev/ttyUSB0 put main.py
  sudo ampy --port /dev/ttyUSB0 put lora.mpy
  #sudo ampy --port /dev/ttyUSB0 put main.mpy
fi


