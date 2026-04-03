"""
Minimal ESPHome external component that brings sensor_update.h / .cpp
into the build.  When this component is loaded, ESPHome automatically
compiles every C++ source file that lives in this directory.

Usage in YAML:
  external_components:
    - source:
        type: local
        path: components    # (or type: git for remote packages)
      components: [sensor_update]

  sensor_update:             # activates the component (empty config is fine)
"""

import esphome.codegen as cg
import esphome.config_validation as cv

CODEOWNERS = []

# A trivial C++ class so ESPHome can register a "component".
# The real work is done by the free functions in sensor_update.cpp,
# called from YAML lambdas.
sensor_update_ns = cg.esphome_ns.namespace("sensor_update_comp")
SensorUpdateComponent = sensor_update_ns.class_(
    "SensorUpdateComponent", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SensorUpdateComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[cv.CONF_ID])
    await cg.register_component(var, config)
