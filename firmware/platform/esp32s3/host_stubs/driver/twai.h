#pragma once
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
typedef int gpio_num_t;
typedef enum { TWAI_MODE_NORMAL } twai_mode_t;
typedef enum { TWAI_STATE_STOPPED, TWAI_STATE_RUNNING, TWAI_STATE_BUS_OFF, TWAI_STATE_RECOVERING } twai_state_t;
typedef struct { twai_mode_t mode; gpio_num_t tx_io, rx_io; uint32_t tx_queue_len, rx_queue_len; } twai_general_config_t;
typedef struct { uint32_t brp; } twai_timing_config_t;
typedef struct { uint32_t acceptance_code, acceptance_mask; bool single_filter; } twai_filter_config_t;
typedef struct { uint32_t extd : 1, rtr : 1; uint32_t identifier; uint8_t data_length_code; uint8_t data[8]; } twai_message_t;
typedef struct { twai_state_t state; uint32_t tx_error_counter, rx_error_counter; } twai_status_info_t;
#define TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, m) {m, tx, rx, 5, 5}
#define TWAI_TIMING_CONFIG_1MBITS() {4}
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() {0, 0xFFFFFFFF, true}
esp_err_t twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*, const twai_filter_config_t*);
esp_err_t twai_start();
esp_err_t twai_receive(twai_message_t*, uint32_t);
esp_err_t twai_transmit(const twai_message_t*, uint32_t);
esp_err_t twai_get_status_info(twai_status_info_t*);
esp_err_t twai_initiate_recovery();
