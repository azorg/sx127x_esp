#!/bin/sh

if mpy-cross -O3 main.py && \
   mpy-cross -O3 sx127x.py
then
  sudo ampy --port /dev/ttyUSB0 put main.py
  #sudo ampy --port /dev/ttyUSB0 put sx127x.py
  sudo ampy --port /dev/ttyUSB0 put sx127x.mpy
fi

