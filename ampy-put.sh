#!/bin/sh


#mpy-cross -O3 main.py
#sudo ampy --port /dev/ttyUSB0 put main.mpy
sudo ampy --port /dev/ttyUSB0 put main.py

mpy-cross -O3 lora.py
sudo ampy --port /dev/ttyUSB0 put lora.mpy


