#pragma once
#include <stdint.h>
class SPISettings { public: SPISettings(uint32_t, uint8_t, uint8_t) {} };
class SPIClass {
public:
    explicit SPIClass(uint8_t) {}
    void begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss = -1);
    void beginTransaction(SPISettings);
    void endTransaction();
    uint8_t transfer(uint8_t);
    uint16_t transfer16(uint16_t);
};
