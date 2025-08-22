import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart, sensor, switch, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID, ENTITY_CATEGORY_CONFIG, ICON_LIGHTBULB

AUTO_LOAD = ["climate", "uart", "sensor", "switch", "text_sensor"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")
ACHIClimate = ac_hi_ns.class_("ACHIClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice)
ACHILEDTargetSwitch = ac_hi_ns.class_("ACHILEDTargetSwitch", switch.Switch)

CONF_ENABLE_PRESETS = "enable_presets"
CONF_PIPE_TEMPERATURE = "pipe_temperature"
CONF_LED_SWITCH = "led_switch"

# Existing optional sensor keys
CONF_SET_TEMPERATURE = "set_temperature"
CONF_ROOM_TEMPERATURE = "room_temperature"
CONF_WIND = "wind"
CONF_SLEEP_STAGE = "sleep_stage"
CONF_MODE_CODE = "mode_code"
CONF_QUIET = "quiet"
CONF_TURBO = "turbo"
CONF_ECONOMY = "economy"
CONF_SWING_UD = "swing_up_down"
CONF_SWING_LR = "swing_left_right"
CONF_POWER_STATUS = "power_status"
CONF_COMP_FR_SET = "compressor_frequency_set"
CONF_COMP_FR = "compressor_frequency"
CONF_OUTDOOR_TEMP = "outdoor_temperature"
CONF_OUTDOOR_COND_TEMP = "outdoor_condenser_temperature"

# New memory diagnostics sensor keys
CONF_HEAP_FREE = "heap_free"
CONF_HEAP_TOTAL = "heap_total"
CONF_HEAP_USED = "heap_used"
CONF_HEAP_MIN_FREE = "heap_min_free"
CONF_HEAP_MAX_ALLOC = "heap_max_alloc"
CONF_HEAP_FRAGMENTATION = "heap_fragmentation"
CONF_PSRAM_TOTAL = "psram_total"
CONF_PSRAM_FREE = "psram_free"

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(ACHIClimate),
    cv.Optional(CONF_ENABLE_PRESETS, default=True): cv.boolean,

    # Optional sensors (existing)
    cv.Optional(CONF_SET_TEMPERATURE): sensor.sensor_schema(),
    cv.Optional(CONF_ROOM_TEMPERATURE): sensor.sensor_schema(),
    cv.Optional(CONF_WIND): sensor.sensor_schema(),
    cv.Optional(CONF_SLEEP_STAGE): sensor.sensor_schema(),
    cv.Optional(CONF_MODE_CODE): sensor.sensor_schema(),
    cv.Optional(CONF_QUIET): sensor.sensor_schema(),
    cv.Optional(CONF_TURBO): sensor.sensor_schema(),
    cv.Optional(CONF_ECONOMY): sensor.sensor_schema(),
    cv.Optional(CONF_SWING_UD): sensor.sensor_schema(),
    cv.Optional(CONF_SWING_LR): sensor.sensor_schema(),
    cv.Optional(CONF_COMP_FR_SET): sensor.sensor_schema(),
    cv.Optional(CONF_COMP_FR): sensor.sensor_schema(),
    cv.Optional(CONF_OUTDOOR_TEMP): sensor.sensor_schema(),
    cv.Optional(CONF_OUTDOOR_COND_TEMP): sensor.sensor_schema(),
    # Power status is a text sensor ("ON"/"OFF")
    cv.Optional(CONF_POWER_STATUS): text_sensor.text_sensor_schema(),

    # Existing optional sensor and LED switch
    cv.Optional(CONF_PIPE_TEMPERATURE): sensor.sensor_schema(),
    cv.Optional(CONF_LED_SWITCH): switch.switch_schema(
        ACHILEDTargetSwitch,
        icon=ICON_LIGHTBULB,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),

    # New memory diagnostics sensors (all optional)
    cv.Optional(CONF_HEAP_FREE): sensor.sensor_schema(),
    cv.Optional(CONF_HEAP_TOTAL): sensor.sensor_schema(),
    cv.Optional(CONF_HEAP_USED): sensor.sensor_schema(),
    cv.Optional(CONF_HEAP_MIN_FREE): sensor.sensor_schema(),
    cv.Optional(CONF_HEAP_MAX_ALLOC): sensor.sensor_schema(),
    cv.Optional(CONF_HEAP_FRAGMENTATION): sensor.sensor_schema(),
    cv.Optional(CONF_PSRAM_TOTAL): sensor.sensor_schema(),
    cv.Optional(CONF_PSRAM_FREE): sensor.sensor_schema(),

}).extend(uart.UART_DEVICE_SCHEMA).extend(cv.polling_component_schema("1s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_comp))

    cg.add(var.set_enable_presets(config[CONF_ENABLE_PRESETS]))

    # Optional numeric sensors (existing)
    if conf := config.get(CONF_SET_TEMPERATURE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_set_temperature_sensor(sens))

    if conf := config.get(CONF_ROOM_TEMPERATURE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_room_temperature_sensor(sens))

    if conf := config.get(CONF_WIND):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_wind_sensor(sens))

    if conf := config.get(CONF_SLEEP_STAGE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_sleep_stage_sensor(sens))

    if conf := config.get(CONF_MODE_CODE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_mode_code_sensor(sens))

    if conf := config.get(CONF_QUIET):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_quiet_sensor(sens))

    if conf := config.get(CONF_TURBO):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_turbo_sensor(sens))

    if conf := config.get(CONF_ECONOMY):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_economy_sensor(sens))

    if conf := config.get(CONF_SWING_UD):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_swing_ud_sensor(sens))

    if conf := config.get(CONF_SWING_LR):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_swing_lr_sensor(sens))

    if conf := config.get(CONF_COMP_FR_SET):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compr_freq_set_sensor(sens))

    if conf := config.get(CONF_COMP_FR):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compr_freq_sensor(sens))

    if conf := config.get(CONF_OUTDOOR_TEMP):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_temp_sensor(sens))

    if conf := config.get(CONF_OUTDOOR_COND_TEMP):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_cond_temp_sensor(sens))

    # Optional text sensor for power status
    if conf := config.get(CONF_POWER_STATUS):
        ts = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_power_status_text(ts))

    # Existing optional sensor: pipe temperature
    if pipe := config.get(CONF_PIPE_TEMPERATURE):
        sens = await sensor.new_sensor(pipe)
        cg.add(var.set_pipe_sensor(sens))

    # Optional LED target switch
    if led_sw_conf := config.get(CONF_LED_SWITCH):
        led_sw = await switch.new_switch(led_sw_conf)
        cg.add(var.set_led_switch(led_sw))

    # New memory diagnostics sensors (optional)
    if conf := config.get(CONF_HEAP_FREE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_heap_free_sensor(sens))

    if conf := config.get(CONF_HEAP_TOTAL):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_heap_total_sensor(sens))

    if conf := config.get(CONF_HEAP_USED):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_heap_used_sensor(sens))

    if conf := config.get(CONF_HEAP_MIN_FREE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_heap_min_free_sensor(sens))

    if conf := config.get(CONF_HEAP_MAX_ALLOC):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_heap_max_alloc_sensor(sens))

    if conf := config.get(CONF_HEAP_FRAGMENTATION):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_heap_fragmentation_sensor(sens))

    if conf := config.get(CONF_PSRAM_TOTAL):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_psram_total_sensor(sens))

    if conf := config.get(CONF_PSRAM_FREE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_psram_free_sensor(sens))
