/* 
   MS5637.cpp: Implementation of MS5637 class library

   Copyright (C) 2018 Simon D. Levy

   Additional dependencies:

       https://github.com/simondlevy/CrossPlatformDataBus

   Adapted from:

       https://raw.githubusercontent.com/kriswiner/MPU9250/master/MPU9250_MS5637_AHRS_t3.ino

   This file is part of MS5637.

   MS5637 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MS5637 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with MS5637.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>   
#include <CrossPlatformI2C_Core.h>

#include "MS5637.h"

// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
static uint8_t MS5637_RESET   =  0x1E;

// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
static uint8_t MS5637_ADDRESS = 0x76;   // Address of altimeter

void MS5637::begin(uint16_t Pcal[8])
{
    cpi2c_writeRegister(MS5637_ADDRESS, 0x00, MS5637_RESET);

    delay(100);

    promRead(Pcal);
}

void MS5637::promRead(uint16_t * destination)
{
    uint8_t data[2] = {0,0};

    for (uint8_t ii = 0; ii < 7; ii++) {

        cpi2c_readRegisters(MS5637_ADDRESS, 0xA0 | ii << 1, 2, data);

        destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); 
    }
}

uint32_t MS5637::read(uint8_t CMD, uint8_t OSR)  
{
    cpi2c_writeRegister(MS5637_ADDRESS, CMD|OSR, 0x00);

    // Delay for conversion to complete
    switch (OSR) {
        case ADC_256: delay(1); break;  
        case ADC_512: delay(3); break;
        case ADC_1024: delay(4); break;
        case ADC_2048: delay(6); break;
        case ADC_4096: delay(10); break;
        case ADC_8192: delay(20); break;
    }

    uint8_t data[3] = {0,0,0};

    cpi2c_readRegisters(MS5637_ADDRESS, 0x00, 3, data);

    return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); 
}

uint8_t MS5637::checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
    int cnt;
    unsigned int n_rem = 0;
    uint8_t n_bit;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
    n_prom[7] = 0;
    for(cnt = 0; cnt < 16; cnt++)
    {
        if(cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else         n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
        for(n_bit = 8; n_bit > 0; n_bit--)
        {
            if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
            else                  n_rem = (n_rem<<1);
        }
    }
    n_rem = ((n_rem>>12) & 0x000F);
    return (n_rem ^ 0x00);
}
