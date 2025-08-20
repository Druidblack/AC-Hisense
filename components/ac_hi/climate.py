import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart, sensor, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID

AUTO_LOAD = ["climate", "uart", "sensor", "text_sensor"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")
ACHIClimate = ac_hi_ns.class_("ACHIClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice)

# Options
CONF_ENABLE_PRESETS = "enable_presets"

# Numeric sensors (Legacy set)
CONF_PIPE_TEMPERATURE = "pipe_temperature"
CONF_SET_TEMPERATURE = "set_temperature"
CONF_ROOM_TEMPERATURE = "room_temperature"
CONF_WIND = "wind"
CONF_SLEEP_STAGE = "sleep_stage"
CONF_MODE_CODE = "mode_code"
CONF_QUIET = "quiet"
CONF_TURBO = "turbo"
CONF_LED = "led"
CONF_ECO = "economy"
CONF_SWING_UP_DOWN = "swing_up_down"
CONF_SWING_LEFT_RIGHT = "swing_left_right"

# Text sensors
CONF_POWER_STATUS = "power_status"

# New: outdoor / compressor metrics from Legacy example
CONF_COMPRESSOR_FREQUENCY_SET = "compressor_frequency_set"
CONF_COMPRESSOR_FREQUENCY = "compressor_frequency"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_OUTDOOR_CONDENSER_TEMPERATURE = "outdoor_condenser_temperature"

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(ACHIClimate),
    cv.Optional(CONF_ENABLE_PRESETS, default=True): cv.boolean,

    # Numeric sensors (all optional)
    cv.Optional(CONF_PIPE_TEMPERATURE): sensor.sensor_schema(),
    cv.Optional(CONF_SET_TEMPERATURE): sensor.sensor_schema(),
    cv.Optional(CONF_ROOM_TEMPERATURE): sensor.sensor_schema(),
    cv.Optional(CONF_WIND): sensor.sensor_schema(),
    cv.Optional(CONF_SLEEP_STAGE): sensor.sensor_schema(),
    cv.Optional(CONF_MODE_CODE): sensor.sensor_schema(),
    cv.Optional(CONF_QUIET): sensor.sensor_schema(),
    cv.Optional(CONF_TURBO): sensor.sensor_schema(),
    cv.Optional(CONF_LED): sensor.sensor_schema(),
    cv.Optional(CONF_ECO): sensor.sensor_schema(),
    cv.Optional(CONF_SWING_UP_DOWN): sensor.sensor_schema(),
    cv.Optional(CONF_SWING_LEFT_RIGHT): sensor.sensor_schema(),

    # Outdoor / compressor
    cv.Optional(CONF_COMPRESSOR_FREQUENCY_SET): sensor.sensor_schema(),
    cv.Optional(CONF_COMPRESSOR_FREQUENCY): sensor.sensor_schema(),
    cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(),
    cv.Optional(CONF_OUTDOOR_CONDENSER_TEMPERATURE): sensor.sensor_schema(),

    # Text sensors
    cv.Optional(CONF_POWER_STATUS): text_sensor.text_sensor_schema(),
}).extend(uart.UART_DEVICE_SCHEMA).extend(cv.polling_component_schema("3s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_comp))
    cg.add(var.set_enable_presets(config[CONF_ENABLE_PRESETS]))

    # Numeric sensors
    if pipe := config.get(CONF_PIPE_TEMPERATURE):
        sens = await sensor.new_sensor(pipe)
        cg.add(var.set_pipe_sensor(sens))

    if st := config.get(CONF_SET_TEMPERATURE):
        sens = await sensor.new_sensor(st)
        cg.add(var.set_set_temp_sensor(sens))

    if rt := config.get(CONF_ROOM_TEMPERATURE):
        sens = await sensor.new_sensor(rt)
        cg.add(var.set_room_temp_sensor(sens))

    if wind := config.get(CONF_WIND):
        sens = await sensor.new_sensor(wind)
        cg.add(var.set_wind_sensor(sens))

    if sl := config.get(CONF_SLEEP_STAGE):
        sens = await sensor.new_sensor(sl)
        cg.add(var.set_sleep_sensor(sens))

    if mode := config.get(CONF_MODE_CODE):
        sens = await sensor.new_sensor(mode)
        cg.add(var.set_mode_sensor(sens))

    if q := config.get(CONF_QUIET):
        sens = await sensor.new_sensor(q)
        cg.add(var.set_quiet_sensor(sens))

    if t := config.get(CONF_TURBO):
        sens = await sensor.new_sensor(t)
        cg.add(var.set_turbo_sensor(sens))

    if l := config.get(CONF_LED):
        sens = await sensor.new_sensor(l)
        cg.add(var.set_led_sensor(sens))

    if e := config.get(CONF_ECO):
        sens = await sensor.new_sensor(e)
        cg.add(var.set_eco_sensor(sens))

    if sud := config.get(CONF_SWING_UP_DOWN):
        sens = await sensor.new_sensor(sud)
        cg.add(var.set_swing_updown_sensor(sens))

    if slr := config.get(CONF_SWING_LEFT_RIGHT):
        sens = await sensor.new_sensor(slr)
        cg.add(var.set_swing_leftright_sensor(sens))

    # Outdoor / compressor
    if cf_set := config.get(CONF_COMPRESSOR_FREQUENCY_SET):
        sens = await sensor.new_sensor(cf_set)
        cg.add(var.set_compr_freq_set_sensor(sens))

    if cf := config.get(CONF_COMPRESSOR_FREQUENCY):
        sens = await sensor.new_sensor(cf)
        cg.add(var.set_compr_freq_sensor(sens))

    if tout := config.get(CONF_OUTDOOR_TEMPERATURE):
        sens = await sensor.new_sensor(tout)
        cg.add(var.set_outdoor_temp_sensor(sens))

    if tcond := config.get(CONF_OUTDOOR_CONDENSER_TEMPERATURE):
        sens = await sensor.new_sensor(tcond)
        cg.add(var.set_outdoor_cond_temp_sensor(sens))

    # Text sensors
    if ps := config.get(CONF_POWER_STATUS):
        ts = await text_sensor.new_text_sensor(ps)
        cg.add(var.set_power_status_text_sensor(ts))