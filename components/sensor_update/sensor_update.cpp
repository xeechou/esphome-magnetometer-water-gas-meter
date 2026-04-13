#include "sensor_update.h"
#include "sensor_update_api.h"
#include <cmath>

// #define DEV_DEBUG
// #define LED_DEBUG

static const char* const TAG = "sensor_update";

// ---------- persistent state (was `static` locals in the lambda) ----------
// Quadrature decoder lookup table
// See:
// https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf

// clang-format off

//values in the matrix:
//  0 : still, not moving
//  1 : moving to clockwise
// -1 : moving to counter-clockwise
//  2 : both bits changing, should absolute not exist.

// NOTE: that because we use [s0 + s1 << 1], we get reversed pattern
//  cw:  [00 -> 01 -> 11 -> 10] or [0 -> 1 -> 3 -> 2]
// ccw:  [00 -> 10 -> 11 -> 01] or [0 -> 2 -> 3 -> 1]

//| p/c |  0  ||  1  ||  2 ||  3 ||
// ------------------------------
//|  0  |  0   |  cw |  ccw |   x |
//|   1 | ccw  |  0  |   x  |  cw |
//|   2 | cw   |   x |   0  | ccw |
//|   3 |  x   | ccw | cw   |   0 |

static const int qdec[16] = {
   0,  1, -1,  2,
  -1,  0,  2,  1,
   1,  2,  0, -1,
   2, -1,  1,  0
};
//clang-format on

static bool  q[2]                = {false, false};
static int   sensor_inversion[3] = {0, 0, 0};
static int   last_sample[2]      = {0, 0};
static float track_min[2]        = {NAN, NAN};
static float track_max[2]        = {NAN, NAN};

// ---------- cached copies set once by sensor_update_init() ----------------

static SensorUpdateConfig s_cfg;
static SensorUpdateEntities s_ent;
static float s_sample_rate_thresh = 0;

void sensor_update_init(const SensorUpdateConfig &cfg,
                        const SensorUpdateEntities &ent)
{
  s_cfg = cfg;
  s_ent = ent;
  //cfg.sensor_update_interval usually 5ms. 1000/5  = 200 steps
  s_sample_rate_thresh =
      1000.0f / cfg.sensor_update_interval_ms * cfg.sample_rate_safety_factor;
}

void sensor_update_reset_trackers(int sensor) {
  track_min[sensor] = NAN;
  track_max[sensor] = NAN;
}

float sensor_update_get_track_min(int sensor) { return track_min[sensor]; }
float sensor_update_get_track_max(int sensor) { return track_max[sensor]; }

static inline bool sensor_high(float value, float tare, float hysteresis)
{
  return value > (tare + hysteresis);
}

static inline bool sensor_low(float value, float tare, float hysteresis)
{
  return value < (tare - hysteresis);
}

static inline bool low_to_high(float value, float tare, float hysteresis,
                               bool last_val)
{
  return sensor_high(value, tare, hysteresis) && !last_val;
}

static inline bool high_to_low(float value, float tare, float hysteresis,
                               bool last_val)
{
  return sensor_low(value, tare, hysteresis) && last_val;
}

// each sample should be alive for at least 2 sensor_update() run because we
// are not able to determine qlast[1-sensor].
static inline bool update_too_fast(int sample_time, int last_sample_time)
{
  return (sample_time - last_sample_time) < 2 * (int)s_cfg.sensor_update_interval_ms;
}

void sensor_update(int sensor, float value) {
  float  magnet_span = s_ent.magnet_span[sensor];
  float &tmin        = track_min[sensor];
  float &tmax        = track_max[sensor];

  esphome::switch_::Switch*  leds[2] = {s_ent.debug_led_0, s_ent.debug_led_1};

  // Not yet calibrated — wait for magnet_span to be set by calibration button
  if (magnet_span <= 0) return;

  // --- spike rejection -------------------------------------------------------
  // Skip values more than one magnet_span outside the current tracker window.
  // Protects against QMC read-during-conversion glitches (large negative spikes
  // seen in recorded data).
  if (!std::isnan(tmin) && value < tmin - magnet_span) return;
  if (!std::isnan(tmax) && value > tmax + magnet_span) return;

  // --- adaptive tracker ------------------------------------------------------
  // Initialize on first reading after boot or after sensor_update_reset_trackers()
  if (std::isnan(tmin) || std::isnan(tmax)) {
    tmax = value + magnet_span / 2.0f;
    tmin = value - magnet_span / 2.0f;
  }

  // Decay inward each sample to follow slow thermal drift
  tmax -= s_cfg.tracker_decay_rate;
  tmin += s_cfg.tracker_decay_rate;

  // Expand when signal exceeds current bounds, clamped to max_span_multiplier
  float max_allowed = magnet_span * s_cfg.max_span_multiplier;
  if (value > tmax) {
    tmax = value;
    if (tmax - tmin > max_allowed) tmin = tmax - max_allowed;
  }
  if (value < tmin) {
    tmin = value;
    if (tmax - tmin > max_allowed) tmax = tmin + max_allowed;
  }

  // Enforce minimum span: prevents window from collapsing to zero when flow stops
  float min_span = magnet_span * s_cfg.min_span_multiplier;
  float span     = tmax - tmin;
  if (span < min_span) {
    float correction = (min_span - span) / 2.0f;
    tmax += correction;
    tmin -= correction;
  }

  s_ent.samples[sensor] += 1;
  // Tare and hysteresis derived from live tracker (hysteresis = 25% of window)
  float tare       = (tmax + tmin) / 2.0f;
  float hysteresis = (tmax - tmin) * 0.2f;

  // --- quadrature decoding ---------------------------------------------------
  // Correctness relies on the single-transition invariant: when sensor_update(s, v)
  // is called, only sensor s has just crossed a threshold. The other sensor (1-s)
  // has not transitioned, so q[1-s] is simultaneously its previous and current
  // state — no separate qlast[] is needed.
  int qlast = q[sensor];
#ifdef DEV_DEBUG
  bool changed = false;
#endif
  if (low_to_high(value, tare, hysteresis, q[sensor])) {
    q[sensor] = true;
#ifdef DEV_DEBUG
    // changed = true;
    // PUBLISH_STATE(qmc_stats[sensor], true);
    // LOG_I(TAG, "sensor %d went from 0 to 1", sensor);
#endif
#ifdef LED_DEBUG
    SWITCH_ON(leds[sensor]);
#endif
  } else if (high_to_low(value, tare, hysteresis, q[sensor])) {
    q[sensor] = false;
#ifdef DEV_DEBUG
    changed = true;
    // PUBLISH_STATE(qmc_stats[sensor], false);
    // LOG_I(TAG, "sensor %d went from 1 to 0", sensor);
#endif
#ifdef LED_DEBUG
    SWITCH_OFF(leds[sensor]);
#endif
  }
  int sample_time = GET_MILLIS();
  // NOTE that if nothing changes, (curr == last) we get qdec[4 * n + n] == 0
  // all the time.
  //
  // also NOTE that we would never get 2 here since qlast[1-sensor] =
  //q[sensor] all the time.
  int curr = (q[sensor] << sensor)  + (q[1-sensor] << (1-sensor));
  int prev = (qlast << sensor)      + (q[1-sensor] << (1-sensor));
  int mod  = qdec[prev * 4 + curr];
#ifdef DEV_DEBUG
  if (mod != 0) {
    PUBLISH_STATE(s_ent.rot_dir, mod);
    // LOG_I(TAG, "sensor %d at %d, (%.2f, %.2f, %.2f) => (%d, %d -> %d)",
    // 	  sensor, sample_time, value, tmin, tmax, prev, curr, mod);
  }
#endif

    if (mod) {
      if (s_cfg.reverse_flow) {
	mod = -mod;
      }
      (*s_ent.quarter_rotations_total) += mod;
    }

  // --- burstmon (sensor inversion test) ------------------------------------
  // bool armed = (*s_ent.disarm_s) == 0;


  if ((mod != 0) && update_too_fast(sample_time, last_sample[sensor])
      // TODO: we don't have sample rate now.
      // && GET_STATE(s_ent.sensor_sample_rate) > s_sample_rate_thresh
      // && armed
      ) {
	// LOG_I(TAG, "reverse flow detected");
	sensor_inversion[sensor]++;
	sensor_inversion[2]++;
	bool any_burst =
          s_cfg.burstmon_total_inversions &&
	  sensor_inversion[2] > s_cfg.burstmon_total_inversions;
	bool same_burst =
          s_cfg.burstmon_sensor_inversions &&
	  sensor_inversion[sensor] > s_cfg.burstmon_sensor_inversions;

	if (any_burst || same_burst) {
	  PUBLISH_STATE(s_ent.leak_warning, true);
	  PUBLISH_STATE(s_ent.leak_detected, true);
	  
	  if (any_burst && same_burst) {
            PUBLISH_STATE(s_ent.leak_trigger, "burstmon: same + any");
	  } else if (any_burst) {
            PUBLISH_STATE(s_ent.leak_trigger, "burstmon: any");
	  } else {
            PUBLISH_STATE(s_ent.leak_trigger, "burstmon: same");
	  }
	}
      } else {
	sensor_inversion[sensor] = 0;
	sensor_inversion[2] = sensor_inversion[0] + sensor_inversion[1]; //force in sync
      }
  last_sample[sensor] = sample_time;
}
