#pragma once

#include "esphome/core/component.h"
#include "sensor_update.h"
#include "sensor_update_api.h"

namespace sensor_update_comp
{

/// Trivial component whose sole purpose is to make ESPHome compile
/// every .cpp file in this directory.  All real logic lives in
/// sensor_update.h / sensor_update.cpp (free functions).
class SensorUpdateComponent : public esphome::Component {
 public:
  void setup() override {}
  void loop() override {}
  float get_setup_priority() const override {
    return esphome::setup_priority::DATA;
  }
};

} // namespace sensor_update_comp
