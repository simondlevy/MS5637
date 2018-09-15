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

MS5637::MS5637(Rate_t osr)
{
    _osr = osr;
}

bool MS5637::begin(void)
{
    cpi2c_writeRegister(MS5637_ADDRESS, 0x00, MS5637_RESET);

    delay(100);

    promRead(_pcal);

    uint8_t refCRC = _pcal[0] >> 12;

    uint8_t nCRC = checkCRC(_pcal);  //calculate checksum to ensure integrity of MS5637 calibration data

    return nCRC == refCRC;
}

void MS5637::readData(double & Temperature, double & Pressure)
{
    static double T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
    uint32_t D1 = read(MS5637::ADC_D1);  // get raw pressure value
    uint32_t D2 = read(MS5637::ADC_D2);  // get raw temperature value
    double dT = D2 - _pcal[5]*pow(2,8);    // calculate temperature difference from reference
    double OFFSET = _pcal[2]*pow(2, 17) + dT*_pcal[4]/pow(2,6);
    double SENS = _pcal[1]*pow(2,16) + dT*_pcal[3]/pow(2,7);

    Temperature = (2000 + (dT*_pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
    //
    // Second order corrections
    if(Temperature > 20) 
    {
        T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
        OFFSET2 = 0;
        SENS2 = 0;
    }
    if(Temperature < 20)                   // correction for low temperature
    {
        T2      = 3*dT*dT/pow(2, 33); 
        OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
        SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
    } 
    if(Temperature < -15)                      // correction for very low temperature
    {
        OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
        SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
    }
    // End of second order corrections
    //
    Temperature = Temperature - T2/100;
    OFFSET = OFFSET - OFFSET2;
    SENS = SENS - SENS2;

    Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa
}

void MS5637::promRead(uint16_t * destination)
{
    uint8_t data[2] = {0,0};

    for (uint8_t ii = 0; ii < 7; ii++) {

        cpi2c_readRegisters(MS5637_ADDRESS, 0xA0 | ii << 1, 2, data);

        destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); 
    }
}

uint32_t MS5637::read(uint8_t cmd)  
{
    cpi2c_writeRegister(MS5637_ADDRESS, cmd|_osr, 0x00);

    // Delay for conversion to complete
    switch (_osr) {
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
