#pragma once

#ifdef MOCK_MODE // compiled on host machine, for testing purpose
#    include <stdint.h>
#    include <algorithm>
#    include <string>

// Mock declarations
// We use void* for entities in mock mode to avoid needing ESPHome headers

void     mock_publish_state(void* entity, float value);
void     mock_publish_state(void* entity, const char* value);
float    mock_get_state(void* entity);
uint32_t mock_millis();
void     mock_log_w(const char* tag, const char* fmt, ...);
void     mock_log_i(const char* tag, const char* fmt, ...);
std::string mock_str_sprintf(const char* fmt, ...);

#    define PUBLISH_STATE(entity, value) mock_publish_state(entity, value)
#    define GET_STATE(entity) mock_get_state(entity)
#    define GET_MILLIS() mock_millis()
#    define LOG_W(tag, fmt, ...) mock_log_w(tag, fmt, ##__VA_ARGS__)
#    define LOG_I(tag, fmt, ...) mock_log_i(tag, fmt, ##__VA_ARGS__)
#    define STR_SPRINTF(fmt, ...) mock_str_sprintf(fmt, ##__VA_ARGS__)

#else // compiled with esphome compile

#    include "esphome/components/binary_sensor/binary_sensor.h"
#    include "esphome/components/sensor/sensor.h"
#    include "esphome/components/text_sensor/text_sensor.h"
#    include "esphome/core/hal.h"
#    include "esphome/core/helpers.h"
#    include "esphome/core/log.h"

#    define PUBLISH_STATE(entity, value) (entity)->publish_state(value)
#    define GET_STATE(entity) (entity)->get_state()
#    define GET_MILLIS() esphome::millis()
#    define LOG_W(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#    define LOG_I(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#    define STR_SPRINTF(fmt, ...) esphome::str_sprintf(fmt, ##__VA_ARGS__)
#endif
