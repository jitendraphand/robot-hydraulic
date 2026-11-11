// Hardware abstraction for the HX-1 ESP32-S3 nodes.
// Keeps all register/driver details out of the control code so the same
// core runs on the host (unit tests, SIL) and on the robot.
#pragma once
#include <stdint.h>

namespace hx {

struct CanFrame {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
};

class Hal {
public:
    // limb: 6 encoders, 16 ADC channels, 8 PWM; hub: 0 encoders, 8 ADC, PWM for ESC/neck/fan
    bool begin(bool hub);
    // AS5048A frames (raw 16-bit) for each encoder chip select
    void read_encoders(uint16_t* frames, int n);
    // MCP3208 channels (12-bit)
    void read_adc(uint16_t* raw, int n);
    // servo-style pulse on an LEDC channel (0 = output low / no pulses)
    void set_pulse_us(int ch, float us);
    void set_gpio(uint8_t pin, bool v);
    bool estop_line_ok();
    bool can_receive(CanFrame& f);
    bool can_send(const CanFrame& f);
    uint32_t can_errors();
    void led(bool on);
    uint32_t micros32();
    // hub-only analog inputs (internal ADC1, millivolts)
    uint32_t analog_mv(uint8_t pin);
    bool digital_in(uint8_t pin);

private:
    bool hub_ = false;
    void spi_cs(uint8_t pin, bool active);
};

}  // namespace hx
