#include "Wire.h"   

#include <CrossPlatformI2C_Core.h>

static const uint8_t ADC_256  = 0x00; // define pressure and temperature conversion rates
static const uint8_t ADC_512  = 0x02;
static const uint8_t ADC_1024 = 0x04;
static const uint8_t ADC_2048 = 0x06;
static const uint8_t ADC_4096 = 0x08;
static const uint8_t ADC_8192 = 0x0A;
static const uint8_t ADC_D1   = 0x40;
static const uint8_t ADC_D2   = 0x50;

static uint8_t OSR = ADC_8192;     // set pressure amd temperature oversample rate

// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
static uint8_t MS5637_RESET   =  0x1E;

// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
static uint8_t MS5637_ADDRESS = 0x76;   // Address of altimeter

static uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
static uint8_t nCRC;       // calculated check sum to ensure PROM integrity
static uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
static double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data

static double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature

// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers

static void MS5637Reset()
{
    cpi2c_writeRegister(MS5637_ADDRESS, 0x00, MS5637_RESET);
}

static void MS5637PromRead(uint16_t * destination)
{
    uint8_t data[2] = {0,0};

    for (uint8_t ii = 0; ii < 7; ii++) {

        cpi2c_readRegisters(MS5637_ADDRESS, 0xA0 | ii << 1, 2, data);

        destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); 
    }
}

static uint32_t MS5637Read(uint8_t CMD, uint8_t OSR)  
{
    Wire.beginTransmission(MS5637_ADDRESS); 
    Wire.write(CMD | OSR);                 
    Wire.endTransmission(false);        

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



static uint8_t MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
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

// ===========================================================================

void setup()
{
    Wire.begin();
    delay(100);
    Serial.begin(115200);

    // Reset the MS5637 pressure sensor
    MS5637Reset();

    delay(100);
    Serial.println("MS5637 pressure sensor reset...");

    // Read PROM data from MS5637 pressure sensor
    MS5637PromRead(Pcal);
    Serial.println("PROM dta read:");
    Serial.print("C0 = ");
    Serial.println(Pcal[0]);
    uint8_t refCRC = Pcal[0] >> 12;
    Serial.print("C1 = ");
    Serial.println(Pcal[1]);
    Serial.print("C2 = ");
    Serial.println(Pcal[2]);
    Serial.print("C3 = ");
    Serial.println(Pcal[3]);
    Serial.print("C4 = ");
    Serial.println(Pcal[4]);
    Serial.print("C5 = ");
    Serial.println(Pcal[5]);
    Serial.print("C6 = ");
    Serial.println(Pcal[6]);

    nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
    Serial.print("Checksum = ");
    Serial.print(nCRC);
    Serial.print(" , should be ");
    Serial.println(refCRC);  
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

        D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
        D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
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
