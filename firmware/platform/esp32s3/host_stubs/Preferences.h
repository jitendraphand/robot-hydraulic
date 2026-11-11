#pragma once
#include <stddef.h>
#include <stdint.h>
class Preferences {
public:
    bool begin(const char*, bool);
    float getFloat(const char*, float);
    size_t putFloat(const char*, float);
    int8_t getChar(const char*, int8_t);
    size_t putChar(const char*, int8_t);
};
