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

    void reset();

    void promRead(uint16_t * destination);

    uint32_t read(uint8_t CMD, uint8_t OSR);  

    uint8_t checkCRC(uint16_t * n_prom);  // calculate checksum from PROM register contents

};
