import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart, sensor, switch
from esphome.const import CONF_ID, CONF_UART_ID, ENTITY_CATEGORY_CONFIG, ICON_LIGHTBULB

AUTO_LOAD = ["climate", "uart", "sensor", "switch"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")
ACHIClimate = ac_hi_ns.class_("ACHIClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice)

# Class for LED target switch (declared in C++)
ACHILEDTargetSwitch = ac_hi_ns.class_("ACHILEDTargetSwitch", switch.Switch)

CONF_ENABLE_PRESETS = "enable_presets"
CONF_PIPE_TEMPERATURE = "pipe_temperature"
CONF_LED_SWITCH = "led_switch"

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(ACHIClimate),
    cv.Optional(CONF_ENABLE_PRESETS, default=True): cv.boolean,
    cv.Optional(CONF_PIPE_TEMPERATURE): sensor.sensor_schema(),
    # Class-aware schema: lets us create switch via switch.new_switch(...)
    cv.Optional(CONF_LED_SWITCH): switch.switch_schema(
        ACHILEDTargetSwitch,
        icon=ICON_LIGHTBULB,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
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

    # Optional LED target switch
    if led_sw_conf := config.get(CONF_LED_SWITCH):
        led_sw = await switch.new_switch(led_sw_conf)
        cg.add(var.set_led_switch(led_sw))
