/* 
   SimpleTest.ino: Example sketch for MS5637 barometer

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


#include <CrossPlatformI2C_Core.h>
#include <Wire.h>

#include "MS5637.h"

static uint8_t OSR = MS5637::ADC_8192;     // set pressure amd temperature oversample rate

static uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
static uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
static double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data

MS5637 ms5637;

void setup()
{
    Wire.begin();

    delay(100);

    Serial.begin(115200);

    // Start the sensor
    ms5637.begin(Pcal);

}

void loop()
{  
    static double Temperature, Pressure; 

    static uint32_t lastUpdate;
    static uint32_t delt_t;
    static uint32_t sumCount;
    static float sum;
    static uint32_t count;

    uint32_t Now = micros();
    float deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

        D1 = ms5637.read(MS5637::ADC_D1, OSR);  // get raw pressure value
        D2 = ms5637.read(MS5637::ADC_D2, OSR);  // get raw temperature value
        dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
        OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
        SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);

        Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;           // First-order Temperature in degrees Centigrade
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

        const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

        float baroin = Pressure; // pressure is now in millibars

        // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
        // comparable to weather report pressure
        float part1 = baroin - 0.3; //Part 1 of formula
        const float part2 = 0.0000842288;
        float part3 = pow(part1, 0.190284);
        float part4 = (float)station_elevation_m / part3;
        float part5 = (1.0 + (part2 * part4));
        float part6 = pow(part5, 5.2553026);
        float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
        baroin = altimeter_setting_pressure_mb * 0.02953;

        Serial.print("Digital temperature value = ");
        Serial.print( (float)Temperature, 2);
        Serial.println(" C"); 
        Serial.print("Digital pressure value = ");
        Serial.print((float) Pressure, 2);  
        Serial.println(" mbar\n");
    }

} // loop()
