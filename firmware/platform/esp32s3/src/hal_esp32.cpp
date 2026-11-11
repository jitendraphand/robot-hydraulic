// ESP32-S3 implementation of the HX-1 HAL (Arduino-ESP32 2.0.x on ESP-IDF 4.4).
#include "hal.h"

#include <Arduino.h>
#include <SPI.h>

#include "driver/twai.h"
#include "hx/can_protocol.h"
#include "pins.h"

namespace hx {

namespace P = hx::pins;

// LEDC timer plan (Arduino 2.x: channel pair 2k/2k+1 shares timer k % 4)
//   limb: ch0-5 @ 333 Hz (valve micro-servos + HV digital servo), ch6-7 @ 50 Hz (analog servos)
//   hub : ch0 ESC @ 50 Hz, ch2-3 neck servos @ 50 Hz, ch4 fan @ 25 kHz
static constexpr int kLedcBits = 14;
static float g_freq[8] = {0};

static SPIClass spi(FSPI);
static const SPISettings kEncSpi(8000000, MSBFIRST, SPI_MODE1);   // AS5048A: CPOL=0, CPHA=1
static const SPISettings kAdcSpi(1600000, MSBFIRST, SPI_MODE0);   // MCP3208 @ 3.3 V

static void ledc_init(int ch, uint8_t pin, float hz) {
    ledcSetup(ch, hz, kLedcBits);
    ledcAttachPin(pin, ch);
    ledcWrite(ch, 0);
    g_freq[ch] = hz;
}

bool Hal::begin(bool hub) {
    hub_ = hub;
    if (!hub) {
        spi.begin(P::limb::SPI_SCLK, P::limb::SPI_MISO, P::limb::SPI_MOSI);
        for (uint8_t i = 0; i < P::limb::ENC_CS_COUNT; ++i) { pinMode(P::limb::ENC_CS[i], OUTPUT); digitalWrite(P::limb::ENC_CS[i], HIGH); }
        for (uint8_t i = 0; i < P::limb::ADC_CS_COUNT; ++i) { pinMode(P::limb::ADC_CS[i], OUTPUT); digitalWrite(P::limb::ADC_CS[i], HIGH); }
        for (uint8_t i = 0; i < P::limb::PWM_COUNT; ++i) ledc_init(i, P::limb::PWM[i], i < 6 ? 333.f : 50.f);
        pinMode(P::limb::ESTOP_SENSE, INPUT);       // external 10k pull-up, opto pulls low when actuator bus is live
        pinMode(P::limb::LED, OUTPUT);
    } else {
        spi.begin(P::hub::SPI_SCLK, P::hub::SPI_MISO, P::hub::SPI_MOSI);
        pinMode(P::hub::ADC_CS[0], OUTPUT);
        digitalWrite(P::hub::ADC_CS[0], HIGH);
        ledc_init(0, P::hub::ESC_PWM, 50.f);
        ledc_init(2, P::hub::NECK_PWM[0], 50.f);
        ledc_init(3, P::hub::NECK_PWM[1], 50.f);
        ledc_init(4, P::hub::FAN_PWM, 25000.f);
        pinMode(P::hub::DUMP_VALVE, OUTPUT);
        digitalWrite(P::hub::DUMP_VALVE, LOW);      // de-energised = dump OPEN (fail-safe)
        pinMode(P::hub::LEVEL_SW, INPUT);
        pinMode(P::hub::ESTOP_SENSE, INPUT);
        pinMode(P::hub::LED, OUTPUT);
        analogReadResolution(12);
    }
    const uint8_t tx = hub ? P::hub::CAN_TX : P::limb::CAN_TX;
    const uint8_t rx = hub ? P::hub::CAN_RX : P::limb::CAN_RX;
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx, (gpio_num_t)rx, TWAI_MODE_NORMAL);
    g.rx_queue_len = 64;
    g.tx_queue_len = 32;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    return twai_start() == ESP_OK;
}

void Hal::spi_cs(uint8_t pin, bool active) { digitalWrite(pin, active ? LOW : HIGH); }

void Hal::read_encoders(uint16_t* frames, int n) {
    // AS5048A returns the result of the PREVIOUS command: read angle twice per sample
    // (command 0xFFFF = read angle with parity). Each transfer ~2 us at 8 MHz.
    spi.beginTransaction(kEncSpi);
    for (int i = 0; i < n && i < P::limb::ENC_CS_COUNT; ++i) {
        spi_cs(P::limb::ENC_CS[i], true);
        spi.transfer16(0xFFFF);
        spi_cs(P::limb::ENC_CS[i], false);
        delayMicroseconds(1);
        spi_cs(P::limb::ENC_CS[i], true);
        frames[i] = spi.transfer16(0xFFFF);
        spi_cs(P::limb::ENC_CS[i], false);
    }
    spi.endTransaction();
}

void Hal::read_adc(uint16_t* raw, int n) {
    spi.beginTransaction(kAdcSpi);
    for (int ch = 0; ch < n; ++ch) {
        const uint8_t cs = hub_ ? P::hub::ADC_CS[0] : P::limb::ADC_CS[ch / 8];
        const uint8_t c = ch % 8;
        spi_cs(cs, true);
        spi.transfer(0x06 | (c >> 2));                        // start, single-ended, D2
        const uint8_t hi = spi.transfer((c & 0x03) << 6) & 0x0F;
        const uint8_t lo = spi.transfer(0x00);
        spi_cs(cs, false);
        raw[ch] = (uint16_t)((hi << 8) | lo);
    }
    spi.endTransaction();
}

void Hal::set_pulse_us(int ch, float us) {
    if (ch < 0 || ch > 7 || g_freq[ch] <= 0) return;
    const float period_us = 1e6f / g_freq[ch];
    const float duty = us <= 0 ? 0.f : us / period_us;
    ledcWrite(ch, (uint32_t)(duty * ((1u << kLedcBits) - 1)));
}

void Hal::set_gpio(uint8_t pin, bool v) { digitalWrite(pin, v ? HIGH : LOW); }

bool Hal::estop_line_ok() {
    // opto conducts (input LOW) while the e-stop-switched actuator bus is energised
    return digitalRead(hub_ ? P::hub::ESTOP_SENSE : P::limb::ESTOP_SENSE) == LOW;
}

bool Hal::can_receive(CanFrame& f) {
    twai_message_t m;
    if (twai_receive(&m, 0) != ESP_OK) return false;
    if (m.extd || m.rtr) return false;
    f.id = m.identifier;
    f.len = m.data_length_code;
    memcpy(f.data, m.data, 8);
    return true;
}

bool Hal::can_send(const CanFrame& f) {
    twai_message_t m = {};
    m.identifier = f.id;
    m.data_length_code = f.len;
    memcpy(m.data, f.data, f.len);
    return twai_transmit(&m, 0) == ESP_OK;
}

uint32_t Hal::can_errors() {
    twai_status_info_t s;
    twai_get_status_info(&s);
    if (s.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
    return s.tx_error_counter + s.rx_error_counter;
}

void Hal::led(bool on) { digitalWrite(hub_ ? P::hub::LED : P::limb::LED, on ? HIGH : LOW); }
uint32_t Hal::micros32() { return (uint32_t)micros(); }
uint32_t Hal::analog_mv(uint8_t pin) { return analogReadMilliVolts(pin); }
bool Hal::digital_in(uint8_t pin) { return digitalRead(pin) == HIGH; }

}  // namespace hx
