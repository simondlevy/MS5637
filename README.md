This repository contains Arduino, Raspberry Pi (WiringPi), and Linux i2cdev
libraries and examples for working with the TE Connectivity / Measurement
Specialties MS5637 Low Voltage Barometric Pressure Sensor.

To use this library you will also need our cross-platform support 
[library](https://github.com/simondlevy/CrossPlatformDataBus).

We have tested this EM7180 library on the following hardware:

* Ladybug STM32L4 board from Tlera Corp

* Teensy 3.2, 3.6

* Raspberry Pi 3

* NVIDIA Jetson TX1

The <b>examples</b> directory contains a simple sketch showing how to use this class. As usual, just clone the repo
into your Arduino libraries folder to get started. The class library and
examples were adapted from  [sketch](https://raw.githubusercontent.com/kriswiner/MPU9250/master/MPU9250_MS5637_AHRS_t3.ino) a by Kris Winer.

RaspberryPi users should download and install [WiringPi](http://wiringpi.com/),
then cd to <b>extras/wiringpi</b>, and run <b>make</b>
to build the examples.  You may have to run the example as root: <tt>sudo ./SimpleTest</tt>.

Users of NVIDIA Jetson and other Linux-based boards should install I<sup>2</sup>C support by running the command:
<pre>
  sudo apt-get install libi2c-dev i2c-tools
</pre>
You can then can cd to <b>extras/i2cdev</b>, and run
<b>make</b>. You may have to run the example as root: <tt>sudo ./SimpleTest</tt>.

An asynchronous [version](https://github.com/bmegli/EM7180.git) of this library is also available for Teensy 3.5.

