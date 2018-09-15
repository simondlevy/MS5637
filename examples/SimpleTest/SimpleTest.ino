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


#include "MS5637.h"

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

static MS5637::Rate_t OSR = MS5637::ADC_8192;     // set pressure amd temperature oversample rate

MS5637 ms5637 = MS5637(OSR);

void setup()
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    Serial.begin(115200);

    // Start the sensor
    if (!ms5637.begin()) {
        while (true) {
            Serial.println("Unable to connect to MS5637");
        }
    }
}

void loop()
{  
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

        double Temperature=0, Pressure=0;
        ms5637.readData(Temperature, Pressure);

        const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on Kris's house; convert from feet to meters

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
