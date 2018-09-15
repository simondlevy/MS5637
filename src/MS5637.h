/* 
   MS5637.h: Declaration of MS5637 class library

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

class MS5637 {

    public:

    static constexpr uint8_t ADC_256  = 0x00; // define pressure and temperature conversion rates
    static constexpr uint8_t ADC_512  = 0x02;
    static constexpr uint8_t ADC_1024 = 0x04;
    static constexpr uint8_t ADC_2048 = 0x06;
    static constexpr uint8_t ADC_4096 = 0x08;
    static constexpr uint8_t ADC_8192 = 0x0A;

    static constexpr uint8_t ADC_D1   = 0x40;
    static constexpr uint8_t ADC_D2   = 0x50;

    bool begin(uint16_t Pcal[8]);

    void promRead(uint16_t * destination);

    uint32_t read(uint8_t CMD, uint8_t OSR);  

    uint8_t checkCRC(uint16_t * n_prom);  // calculate checksum from PROM register contents

};
