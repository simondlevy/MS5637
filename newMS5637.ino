#include "Wire.h"   

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

static uint8_t OSR = ADC_8192;     // set pressure amd temperature oversample rate


// See MS5637-02BA03 Low Voltage Barometric Pressure Sensor Data Sheet
#define MS5637_RESET      0x1E
#define MS5637_CONVERT_D1 0x40
#define MS5637_CONVERT_D2 0x50
#define MS5637_ADC_READ   0x00

// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define MS5637_ADDRESS 0x76   // Address of altimeter

uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
uint32_t D1 = 0, D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data

int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature

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
    unsigned char refCRC = Pcal[0] >> 12;
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

        float altitude = 145366.45*(1. - pow((Pressure/1013.25), 0.190284));

        Serial.print("Digital temperature value = ");
        Serial.print( (float)Temperature, 2);
        Serial.println(" C"); // temperature in degrees Celsius
        Serial.print("Digital temperature value = ");
        Serial.print(9.*(float) Temperature/5. + 32., 2);
        Serial.println(" F"); // temperature in degrees Fahrenheit
        Serial.print("Digital pressure value = ");
        Serial.print((float) Pressure, 2);  Serial.println(" mbar");// pressure in millibar
        Serial.print("Altitude = ");
        Serial.print(altitude, 2);
        Serial.println(" feet\n");
    }

} // loop()

// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers

void MS5637Reset()
{
    Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    Wire.write(MS5637_RESET);                // Put reset command in Tx buffer
    Wire.endTransmission();                  // Send the Tx buffer
}

void MS5637PromRead(uint16_t * destination)
{
    uint8_t data[2] = {0,0};
    for (uint8_t ii = 0; ii < 7; ii++) {
        Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
        Wire.write(0xA0 | ii << 1);              // Put PROM address in Tx buffer
        Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
        uint8_t i = 0;
        Wire.requestFrom(MS5637_ADDRESS, 2);   // Read two bytes from slave PROM address 
        while (Wire.available()) {
            data[i++] = Wire.read(); }               // Put read results in the Rx buffer
        destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
    }
}

uint32_t MS5637Read(uint8_t CMD, uint8_t OSR)  // temperature data read
{
    uint8_t data[3] = {0,0,0};
    Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    Wire.write(CMD | OSR);                  // Put pressure conversion command in Tx buffer
    Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive

    switch (OSR)
    {
        case ADC_256: delay(1); break;  // delay for conversion to complete
        case ADC_512: delay(3); break;
        case ADC_1024: delay(4); break;
        case ADC_2048: delay(6); break;
        case ADC_4096: delay(10); break;
        case ADC_8192: delay(20); break;
    }

    Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    Wire.write(0x00);                        // Put ADC read command in Tx buffer
    Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(MS5637_ADDRESS, 3);     // Read three bytes from slave PROM address 
    while (Wire.available()) {
        data[i++] = Wire.read(); }               // Put read results in the Rx buffer
    return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
}



unsigned char MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
    int cnt;
    unsigned int n_rem = 0;
    unsigned char n_bit;

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


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data	 
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);	                 // Put slave register address in Tx buffer
    Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
    //	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    //	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
    Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
    //	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    //        Wire.requestFrom(address, count);  // Read bytes from slave register address 
    Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}
