#pragma once

// Forward declarations for ESPHome entity types we receive as parameters.
// We only need pointers/references to call publish_state() on them,
// so forward decls suffice and avoid pulling in the full ESPHome headers.
namespace esphome {
namespace sensor {
class Sensor;
}  // namespace sensor
namespace binary_sensor {
class BinarySensor;
}  // namespace binary_sensor
namespace text_sensor {
class TextSensor;
}  // namespace text_sensor
}  // namespace esphome

/// Configuration constants passed from YAML substitutions.
struct SensorUpdateConfig {
  float sensor_update_interval_ms;
  float sample_rate_safety_factor;
  bool reverse_flow;
  int burstmon_any_thresh;
  int burstmon_same_thresh;
};

/// Pointers to ESPHome entities that the algorithm needs to interact with.
/// These are set once from the YAML lambda and reused on every call.
struct SensorUpdateEntities {
  // globals (arrays/scalars passed by pointer)
  long *errors;
  int *samples;           // int[2]
  long *quarter_rotations_total;
  long *quarter_rotations_flow;
  float *qcal;            // float[5]
  float *disarm_s;

  // sensors / binary sensors / text sensors
  esphome::sensor::Sensor *sensor_sample_rate;
  esphome::binary_sensor::BinarySensor *leak_detected;
  esphome::binary_sensor::BinarySensor *leak_warning;
  esphome::binary_sensor::BinarySensor *sensor_failure;
  esphome::text_sensor::TextSensor *leak_trigger;
  esphome::text_sensor::TextSensor *hyst_sensor;
};

/// One-time initialisation — call from esphome on_boot or the first lambda.
void sensor_update_init(const SensorUpdateConfig &cfg,
                        const SensorUpdateEntities &ent);

/// Called on every raw sensor reading.  `sensor` is 0 or 1, `value` is the
/// raw magnetometer Z field strength.
void sensor_update(int sensor, float value);
