#include "sensor_update.h"
#include "sensor_update_api.h"

static const char *const TAG = "sensor_update";

// ---------- persistent state (was `static` locals in the lambda) ----------

// Quadrature decoder lookup table
// See: https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
static const int qdec[16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};

static int q[2] = {0, 0};
static int run[3] = {0, 0, 0};
static int hyst_sampling = 0;
static int calib_rots = 0;
static int last_sample[2] = {0, 0};

// ---------- cached copies set once by sensor_update_init() ----------------

static SensorUpdateConfig s_cfg;
static SensorUpdateEntities s_ent;
static float s_sample_rate_thresh = 0;

void sensor_update_init(const SensorUpdateConfig &cfg,
                        const SensorUpdateEntities &ent) {
  s_cfg = cfg;
  s_ent = ent;
  s_sample_rate_thresh =
      1000.0f / cfg.sensor_update_interval_ms * cfg.sample_rate_safety_factor;
}

void sensor_update(int sensor, float value) {
  float &smin = s_ent.qcal[2 * sensor];
  float &smax = s_ent.qcal[2 * sensor + 1];
  float &hysteresis = s_ent.qcal[4];

  s_ent.samples[sensor] += 1;

  // --- auto-calibration ---------------------------------------------------
  if (!smin && !smax) {
    smin = smax = value;
    hyst_sampling = 2000;
    calib_rots = 1000;
  } else if (calib_rots) {
    smin = std::min(smin, value);
    smax = std::max(smax, value);
  }

  // --- hysteresis learning phase ------------------------------------------
  if (hyst_sampling) {
    hyst_sampling--;
    if (smax - smin > hysteresis) {
      hysteresis = smax - smin;
      hyst_sampling = 2000;
      if (hysteresis > 20) {
        hysteresis = 20;
        hyst_sampling = 0;
        PUBLISH_STATE(s_ent.sensor_failure, true);
        LOG_W(TAG, "hysteresis overly high (>%g); is water flowing?",
                 hysteresis);
      }
    }
    PUBLISH_STATE(s_ent.hyst_sensor,
                  STR_SPRINTF("hysteresis set to %g", hysteresis).c_str());
    LOG_I(TAG, STR_SPRINTF("hysteresis set to %g", hysteresis).c_str());
    return;
  }
  else if (smax - smin < 2 * hysteresis)
  {
      return;
  }

  // --- quadrature decoding ------------------------------------------------
  int tare = (smin + smax) / 2;
  int mod = 0;

  if (value > tare + hysteresis && !q[sensor]) {
    q[sensor] = 1;
    // NB: original precedence is (5 << 1) - sensor, not 5 << (1-sensor)
    mod = qdec[q[1 - sensor] * ((5 << 1) - sensor) + (1 << sensor)];
  } else if (value < tare - hysteresis && q[sensor]) {
    q[sensor] = 0;
    mod = qdec[q[1 - sensor] * ((5 << 1) - sensor) + (4 << sensor)];
  }

  if (mod == 2) {
    (*s_ent.errors) += 1;
  } else if (mod) {
    if (calib_rots) {
      calib_rots--;
      if (!calib_rots) {
        LOG_I(TAG, "min/max detected as %g/%g, %g/%g",
                 s_ent.qcal[0], s_ent.qcal[1],
                 s_ent.qcal[2], s_ent.qcal[3]);
      }
    }
    if (s_cfg.reverse_flow) {
      mod = -mod;
    }
    (*s_ent.quarter_rotations_total) += mod;
    (*s_ent.quarter_rotations_flow) += mod;
  }

  // --- burstmon (pipe-burst detection) ------------------------------------
  bool armed = (*s_ent.disarm_s) <= 0;
  int sample_time = GET_MILLIS();

  if (mod
      && sample_time - last_sample[sensor] < 2 * (int)s_cfg.sensor_update_interval_ms
      && GET_STATE(s_ent.sensor_sample_rate) > s_sample_rate_thresh
      && armed) {
    run[sensor]++;
    run[2]++;
    bool any_burst =
        s_cfg.burstmon_any_thresh && run[2] > s_cfg.burstmon_any_thresh;
    bool same_burst =
        s_cfg.burstmon_same_thresh && run[sensor] > s_cfg.burstmon_same_thresh;

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
    run[sensor] = 0;
    run[2] = 0;
  }
  last_sample[sensor] = sample_time;
}
