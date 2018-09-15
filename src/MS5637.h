static const uint8_t ADC_256  = 0x00; // define pressure and temperature conversion rates
static const uint8_t ADC_512  = 0x02;
static const uint8_t ADC_1024 = 0x04;
static const uint8_t ADC_2048 = 0x06;
static const uint8_t ADC_4096 = 0x08;
static const uint8_t ADC_8192 = 0x0A;

static uint8_t OSR = ADC_8192;     // set pressure amd temperature oversample rate
static const uint8_t ADC_D1   = 0x40;
static const uint8_t ADC_D2   = 0x50;

void MS5637Reset();

void MS5637PromRead(uint16_t * destination);

uint32_t MS5637Read(uint8_t CMD, uint8_t OSR);  

uint8_t MS5637checkCRC(uint16_t * n_prom);  // calculate checksum from PROM register contents
