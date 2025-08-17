import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart, sensor
from esphome.const import CONF_ID, CONF_UART_ID

AUTO_LOAD = ["climate", "uart", "sensor"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")
ACHIClimate = ac_hi_ns.class_("ACHIClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice)

CONF_ENABLE_PRESETS = "enable_presets"
CONF_PIPE_TEMPERATURE = "pipe_temperature"

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(ACHIClimate),
    cv.Optional(CONF_ENABLE_PRESETS, default=True): cv.boolean,
    cv.Optional(CONF_PIPE_TEMPERATURE): sensor.sensor_schema(),
}).extend(uart.UART_DEVICE_SCHEMA).extend(cv.polling_component_schema("1s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_comp))
    cg.add(var.set_enable_presets(config[CONF_ENABLE_PRESETS]))
    if pipe := config.get(CONF_PIPE_TEMPERATURE):
        sens = await sensor.new_sensor(pipe)
        cg.add(var.set_pipe_sensor(sens))
