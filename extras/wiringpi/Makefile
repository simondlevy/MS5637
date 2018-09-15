# 
# Makefile for MS5637 WiringPi example
#
# Copyright (c) 2018 Simon D. Levy
#
# This file is part of MS5637.
#
# MS5637 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# MS5637 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with MS5637.  If not, see <http://www.gnu.org/licenses/>.

  
# Change this to whereever you installed CrossPlatformI2C
CPDB = $(HOME)/CrossPlatformDataBus

ALL = SimpleTest

MSSRC = ../../src

I2CINC = $(CPDB)/src
I2CSRC = $(CPDB)/extras/i2c/wiringpi/src/

all: $(ALL)

SimpleTest: SimpleTest.o MS5637.o WiringPiI2C.o main.o
	g++ -std=c++11 -o SimpleTest SimpleTest.o MS5637.o WiringPiI2C.o main.o -lwiringPi

SimpleTest.o: SimpleTest.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MSSRC) -c SimpleTest.cpp

MS5637.o: $(MSSRC)/MS5637.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(MSSRC) -c $(MSSRC)/MS5637.cpp

WiringPiI2C.o: $(I2CSRC)/WiringPiI2C.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -c $(I2CSRC)/WiringPiI2C.cpp

main.o: main.cpp 
	g++ -std=c++11 -Wall -c main.cpp

run: SimpleTest
	./SimpleTest

clean:
	rm -f $(ALL) *.o *~
