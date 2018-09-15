This repository contains Arduino, Raspberry Pi (WiringPi), and Linux i2cdev
libraries and examples for working with the TE Connectivity / Measurement
Specialties MS5637 Low Voltage Barometric Pressure Sensor.

To use this library you will also need our cross-platform support 
[library](https://github.com/simondlevy/CrossPlatformDataBus).

We have tested this MS5637 library on the following hardware:

* Buterfly STM32L4 board from Tlera Corp

* Raspberry Pi 3

* NVIDIA Jetson TX1

The <b>examples</b> directory contains a simple sketch showing how to use this class. As usual, just clone the repo
into your Arduino libraries folder to get started. The class library and
examples were adapted from  [sketch](https://raw.githubusercontent.com/kriswiner/MPU9250/master/MPU9250_MS5637_AHRS_t3.ino) a by Kris Winer.
