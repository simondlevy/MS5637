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


#include "MS5637.h"
#include <CrossPlatformI2C_Core.h>
#include <math.h>

MS5637::MS5637(Rate_t osr)
{
    _osr = osr;
}

MS5637::Error_t MS5637::begin(uint8_t bus)
{
    _i2c = cpi2c_open(ADDRESS, bus);

    if (_i2c <= 0) {
        return ERROR_CONNECT;
    }

    delay(200);

    cpi2c_writeRegister(_i2c, 0x00, RESET);

    delay(200);

    promRead(_pcal);

    uint8_t refCRC = _pcal[0] >> 12;

    uint8_t nCRC = checkCRC(_pcal);  //calculate checksum to ensure integrity of MS5637 calibration data

    if (nCRC != refCRC) {
        return ERROR_CHECKSUM;
    }

    return ERROR_NONE;
}

void MS5637::readData(float & temperature, float & pressure)
{
    static float t2, offset2, sens2;  // First order and second order corrections for raw S5637 temperature and pressure data
    uint32_t d1 = read(MS5637::ADC_D1);  // get raw pressure value
    uint32_t d2 = read(MS5637::ADC_D2);  // get raw temperature value
    float dT = d2 - _pcal[5]*powf(2,8);    // calculate temperature difference from reference
    float offset = _pcal[2]*powf(2, 17) + dT*_pcal[4]/powf(2,6);
    float sens = _pcal[1]*powf(2,16) + dT*_pcal[3]/powf(2,7);

    temperature = (2000 + (dT*_pcal[6])/powf(2, 23))/100;           // First-order temperature in degrees Centigrade
    //
    // Second order corrections
    if(temperature > 20) 
    {
        t2 = 5*dT*dT/powf(2, 38); // correction for high temperatures
        offset2 = 0;
        sens2 = 0;
    }
    if(temperature < 20)                   // correction for low temperature
    {
        t2      = 3*dT*dT/powf(2, 33); 
        offset2 = 61*(100*temperature - 2000)*(100*temperature - 2000)/16;
        sens2   = 29*(100*temperature - 2000)*(100*temperature - 2000)/16;
    } 
    if(temperature < -15)                      // correction for very low temperature
    {
        offset2 = offset2 + 17*(100*temperature + 1500)*(100*temperature + 1500);
        sens2 = sens2 + 9*(100*temperature + 1500)*(100*temperature + 1500);
    }
    // End of second order corrections
    //
    temperature = temperature - t2/100;
    offset = offset - offset2;
    sens = sens - sens2;

    pressure = (((d1*sens)/powf(2, 21) - offset)/powf(2, 15))/100;  // pressure in mbar or kPa
}

void MS5637::promRead(uint16_t * destination)
{
    uint8_t data[2] = {0,0};

    for (uint8_t ii = 0; ii < 7; ii++) {

        cpi2c_readRegisters(_i2c, 0xA0 | ii << 1, 2, data);

        destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); 
    }
}

uint32_t MS5637::read(uint8_t cmd)  
{
    cpi2c_writeRegister(_i2c, cmd|_osr, 0x00);

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

    cpi2c_readRegisters(_i2c, 0x00, 3, data);

    return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); 
}

uint8_t MS5637::checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
    uint8_t cnt;
    uint32_t n_rem = 0;
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
