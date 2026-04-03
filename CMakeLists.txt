cmake_minimum_required(VERSION 3.14)
project(espleak LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


####### externals ############################################################
Include(FetchContent)

##TODO:

####### main project ############################################################

add_library(sensor_update SHARED
  components/sensor_update/sensor_update.h
  components/sensor_update/sensor_update.cpp
  components/sensor_update/sensor_update_api_mock.cpp
)

target_compile_definitions(sensor_update
  PRIVATE
  "MOCK_MODE"
  # ---------------------------------------------------------------------------
  # Platform defines — mimic what ESPHome codegen would produce.
  # These are for IDE analysis only.
  # ---------------------------------------------------------------------------
  USE_ESP32
  USE_ESP32_VARIANT_ESP32S3
  USE_ESP32_FRAMEWORK_ESP_IDF
  USE_ESP_IDF

  # Features used by this project
  USE_SENSOR
  USE_BINARY_SENSOR
  USE_TEXT_SENSOR
  USE_BUTTON
  USE_LOGGER
  USE_API
  USE_WIFI
  USE_OTA
  USE_NETWORK
  USE_OUTPUT
  USE_CAPTIVE_PORTAL
  USE_MDNS
  USE_WEBSERVER

  # Logging level (INFO = 3)
  ESPHOME_LOG_LEVEL=3
)

# This target will not actually compile (missing ESP-IDF toolchain, generated
# files, etc.), but it gives the IDE everything it needs for navigation,
# autocompletion, and diagnostics on our own source files.
