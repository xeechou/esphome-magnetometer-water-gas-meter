cmake_minimum_required(VERSION 3.14)
project(espleak LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Fetch ESPHome source so we get the real headers for IDE intellisense.
# This is NOT used for building firmware — ESPHome/PlatformIO does that.
# It only gives us esphome/core/*.h and esphome/components/*.h.
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
  esphome
  GIT_REPOSITORY https://github.com/esphome/esphome.git
  GIT_TAG        2025.3.3
  GIT_SHALLOW    ON
  GIT_PROGRESS   ON
)
FetchContent_MakeAvailable(esphome)

# The headers live under <esphome_SOURCE_DIR>/esphome/
# e.g.  <esphome_SOURCE_DIR>/esphome/core/application.h
#        <esphome_SOURCE_DIR>/esphome/components/sensor/sensor.h
set(ESPHOME_ROOT "${esphome_SOURCE_DIR}/esphome")

# ---------------------------------------------------------------------------
# Platform defines — mimic what ESPHome codegen would produce.
# These are for IDE analysis only.
# ---------------------------------------------------------------------------
add_compile_definitions(
  # Platform
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

# ---------------------------------------------------------------------------
# INTERFACE library — header-only target for intellisense
# ---------------------------------------------------------------------------
add_library(esphome_headers INTERFACE)
target_include_directories(esphome_headers INTERFACE
  "${esphome_SOURCE_DIR}"            # so  #include "esphome/core/xyz.h"  works
)

# ---------------------------------------------------------------------------
# Our source files — the actual code that ESPHome will compile
# ---------------------------------------------------------------------------
add_library(sensor_update STATIC
  src/sensor_update.h
  src/sensor_update.cpp
)
target_link_libraries(sensor_update PUBLIC esphome_headers)

# This target will not actually compile (missing ESP-IDF toolchain, generated
# files, etc.), but it gives the IDE everything it needs for navigation,
# autocompletion, and diagnostics on our own source files.
