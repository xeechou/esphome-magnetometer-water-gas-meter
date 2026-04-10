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
struct SensorUpdateConfig
{
    float sensor_update_interval_ms; // 5
    float sample_rate_safety_factor; // 0.95
    bool  reverse_flow;
    int   burstmon_total_inversions;  // 3
    int   burstmon_sensor_inversions; // 2
    // Adaptive tracker parameters
    float tracker_decay_rate;  // inward decay per sample (follows thermal drift)
    float min_span_multiplier; // tracker window floor as fraction of magnet_span
    float max_span_multiplier; // tracker window ceiling as fraction of magnet_span
};

/// Pointers to ESPHome entities that the algorithm needs to interact with.
/// These are set once from the YAML lambda and reused on every call.
struct SensorUpdateEntities {
  // globals (arrays/scalars passed by pointer)
  long  *errors;
  int   *samples;                   // int[2]
  long  *quarter_rotations_total;
  long  *quarter_rotations_flow;
  float *magnet_span;               // float[2], persistent calibrated span per sensor
  float *disarm_s;

  // sensors / binary sensors / text sensors
  esphome::sensor::Sensor*              sensor_sample_rate;
  esphome::binary_sensor::BinarySensor* leak_detected;
  esphome::binary_sensor::BinarySensor* leak_warning;
  esphome::binary_sensor::BinarySensor* sensor_failure;
  esphome::text_sensor::TextSensor*     leak_trigger;
  esphome::sensor::Sensor*              rot_dir;
  // esphome::sensor::Sensor*              qmca;
  // esphome::sensor::Sensor*              qmcb;
};

/// One-time initialisation — call from esphome on_boot or the first lambda.
void sensor_update_init(const SensorUpdateConfig &cfg,
                        const SensorUpdateEntities &ent);

/// Called on every raw sensor reading.  `sensor` is 0 or 1, `value` is the
/// raw magnetometer Z field strength.
void sensor_update(int sensor, float value);

/// Reset the adaptive tracker for one sensor (call after writing a new
/// magnet_span so the tracker re-initialises from the next reading).
void sensor_update_reset_trackers(int sensor);

/// Read back the current tracker bounds (NAN if not yet initialized).
float sensor_update_get_track_min(int sensor);
float sensor_update_get_track_max(int sensor);
