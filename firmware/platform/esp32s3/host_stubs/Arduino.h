// Host-only stand-in for Arduino-ESP32 2.0.x: declarations with the same
// signatures, used ONLY to compile-check the node application on CI/host.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
typedef uint32_t TickType_t;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define pdMS_TO_TICKS(x) (x)
#define configMAX_PRIORITIES 25
#define MSBFIRST 1
#define SPI_MODE0 0
#define SPI_MODE1 1
#define FSPI 0
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
void delay(uint32_t);
void delayMicroseconds(uint32_t);
unsigned long micros();
unsigned long millis();
double ledcSetup(uint8_t ch, double freq, uint8_t bits);
void ledcAttachPin(uint8_t pin, uint8_t ch);
void ledcWrite(uint8_t ch, uint32_t duty);
void analogReadResolution(uint8_t);
uint32_t analogReadMilliVolts(uint8_t);
float temperatureRead();
TickType_t xTaskGetTickCount();
void vTaskDelayUntil(TickType_t*, TickType_t);
typedef void (*TaskFunction_t)(void*);
int xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, unsigned, void*, int);
class String : public std::string {
public:
    String() = default;
    String(const std::string& s) : std::string(s) {}
    void trim();
    bool startsWith(const char*) const;
    String substring(unsigned) const;
    long toInt() const;
    bool operator==(const char* o) const { return std::string(*this) == o; }
};
class HardwareSerial {
public:
    void begin(unsigned long);
    int available();
    String readStringUntil(char);
    int printf(const char*, ...);
    size_t println(const char*);
};
extern HardwareSerial Serial;
class TwoWire { public: bool begin(int sda, int scl, uint32_t freq); };
extern TwoWire Wire;
