#pragma once
#include <stdint.h>
#define SH2_ROTATION_VECTOR 5
struct sh2_RotationVector_t { float real, i, j, k; };
struct sh2_SensorValue_t { uint8_t sensorId; union { sh2_RotationVector_t rotationVector; } un; };
class Adafruit_BNO08x {
public:
    explicit Adafruit_BNO08x(int8_t reset_pin = -1);
    bool begin_I2C();
    bool enableReport(int id, uint32_t interval_us);
    bool getSensorEvent(sh2_SensorValue_t*);
};
