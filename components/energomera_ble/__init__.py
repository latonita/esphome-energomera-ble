import re
from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, binary_sensor, time
from esphome.components import sensor as esphome_sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_PIN,
    CONF_SIGNAL_STRENGTH,
    CONF_TIME_ID,
    CONF_UPDATE_INTERVAL,
    ENTITY_CATEGORY_DIAGNOSTIC,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL_MILLIWATT,
)

CODEOWNERS = ["@latonita"]

AUTO_LOAD = ["binary_sensor"]

DEPENDENCIES = ["ble_client", "sensor"]

MULTI_CONF = True

DEFAULTS_MAX_SENSOR_INDEX = 12
DEFAULTS_UPDATE_INTERVAL = "60s"

CONF_ENERGOMERA_BLE_ID = "energomera_ble_id"
CONF_REQUEST = "request"
CONF_SUB_INDEX = "sub_index"

energomera_ble_ns = cg.esphome_ns.namespace("energomera_ble")
EnergomeraBle = energomera_ble_ns.class_(
    "EnergomeraBleComponent", cg.Component, ble_client.BLEClientNode
)


def validate_request_format(value):
    if not value.endswith(")"):
        value += "()"

    pattern = r"\b[a-zA-Z_][a-zA-Z0-9_]*\s*\(.*\)$"
    if not re.match(pattern, value):
        raise cv.Invalid(
            "Invalid request format. Proper is 'REQUEST' or 'REQUEST()' or 'REQUEST(ARGS)'"
        )

    if len(value) > 60:
        raise cv.Invalid(
            "Request length must be no longer than 60 characters including ()"
        )
    return value

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EnergomeraBle),
            cv.Optional(
                CONF_UPDATE_INTERVAL, default=DEFAULTS_UPDATE_INTERVAL
            ): cv.update_interval,
            cv.Optional(CONF_SIGNAL_STRENGTH): cv.maybe_simple_value(
                esphome_sensor.sensor_schema(
                    unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
                    accuracy_decimals=0,
                    device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
                    state_class=STATE_CLASS_MEASUREMENT,
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),
                key=CONF_NAME,
            ),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Required(CONF_PIN): cv.int_range(min=0,max=999999),
        }
    )
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    if signal_strength_config := config.get(CONF_SIGNAL_STRENGTH):
        sens = await esphome_sensor.new_sensor(signal_strength_config)
        cg.add(var.set_signal_strength(sens))

    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_source(time_))


    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    cg.add(var.set_passkey(config[CONF_PIN]))
