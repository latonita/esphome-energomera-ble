import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_INDEX,
)
from . import (
    EnergomeraBle,
    CONF_ENERGOMERA_BLE_ID,
    energomera_ble_ns,
    validate_request_format,
    CONF_REQUEST,
    CONF_SUB_INDEX,
    DEFAULTS_MAX_SENSOR_INDEX,
)

AUTO_LOAD = ["energomera_ble"]

EnergomeraBleTextSensor = energomera_ble_ns.class_(
    "EnergomeraBleTextSensor", text_sensor.TextSensor
)

CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(
        EnergomeraBleTextSensor,
    ).extend(
        {
            cv.GenerateID(CONF_ENERGOMERA_BLE_ID): cv.use_id(EnergomeraBle),
            cv.Required(CONF_REQUEST): validate_request_format,
            cv.Optional(CONF_INDEX, default=1): cv.int_range(
                min=1, max=DEFAULTS_MAX_SENSOR_INDEX
            ),
            cv.Optional(CONF_SUB_INDEX, default=0): cv.int_range(
                min=0, max=255
            ),
        }
    ),
    cv.has_exactly_one_key(CONF_REQUEST),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_ENERGOMERA_BLE_ID])
    var = await text_sensor.new_text_sensor(config)

    if CONF_REQUEST in config:
        cg.add(var.set_request(config[CONF_REQUEST]))

    cg.add(var.set_index(config[CONF_INDEX]))
    cg.add(var.set_sub_index(config[CONF_SUB_INDEX]))

    cg.add(component.register_sensor(var))
