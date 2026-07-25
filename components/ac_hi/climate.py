import esphome.codegen as cg
from esphome import automation
import esphome.config_validation as cv
from esphome.components import climate, uart, sensor, switch, select, text_sensor, remote_base
from esphome.const import CONF_ID, CONF_UART_ID, CONF_NAME, CONF_TEMPERATURE, ENTITY_CATEGORY_CONFIG, ENTITY_CATEGORY_DIAGNOSTIC, ICON_LIGHTBULB

AUTO_LOAD = ["climate", "uart", "sensor", "switch", "select", "text_sensor", "remote_base"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")
ACHIClimate = ac_hi_ns.class_("ACHIClimate", climate.Climate, cg.PollingComponent, uart.UARTDevice)
ACHILEDTargetSwitch = ac_hi_ns.class_("ACHILEDTargetSwitch", switch.Switch)
ACHICommandSoundSwitch = ac_hi_ns.class_("ACHICommandSoundSwitch", switch.Switch)
ACHIMemorySwitch = ac_hi_ns.class_("ACHIMemorySwitch", switch.Switch)
ACHISleepProgramSelect = ac_hi_ns.class_("ACHISleepProgramSelect", select.Select)
ACHIIFeelAction = ac_hi_ns.class_("ACHIIFeelAction", automation.Action)

# ESPHome 2025.5+ uses climate.climate_schema(...), older versions still use CLIMATE_SCHEMA.
if hasattr(climate, "climate_schema"):
    BASE_CLIMATE_SCHEMA = climate.climate_schema(ACHIClimate)
    BASE_CLIMATE_EXTRA = {}
else:
    BASE_CLIMATE_SCHEMA = climate.CLIMATE_SCHEMA
    BASE_CLIMATE_EXTRA = {
        cv.GenerateID(): cv.declare_id(ACHIClimate),
    }

CONF_ENABLE_PRESETS = "enable_presets"
CONF_PIPE_TEMPERATURE = "pipe_temperature"
CONF_LED_SWITCH = "led_switch"
CONF_SOUND_SWITCH = "sound_switch"
CONF_MEMORY_SWITCH = "memory_switch"
CONF_SLEEP_PROGRAM = "sleep_program"
CONF_IR_TRANSMITTER_ID = "ir_transmitter_id"
CONF_IFEEL_MQTT_TOPIC = "ifeel_mqtt_topic"
CONF_IFEEL_MQTT_PAYLOAD = "ifeel_mqtt_payload"
CONF_IFEEL_MQTT_QOS = "ifeel_mqtt_qos"
CONF_IFEEL_MQTT_RETAIN = "ifeel_mqtt_retain"
CONF_ENABLED = "enabled"

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
CONF_COMP_FR_ACTUAL = "compressor_frequency_actual"
CONF_COMP_FR_SET = "compressor_frequency_set"
CONF_COMP_FR_COMMAND = "compressor_frequency_command"
# Backward-compatible alias for byte 43 used by older YAML configurations.
CONF_COMP_FR = "compressor_frequency"
CONF_OUTDOOR_TEMP = "outdoor_temperature"
CONF_OUTDOOR_COND_TEMP = "outdoor_condenser_temperature"
CONF_COMPRESSOR_EXHAUST_TEMP = "compressor_exhaust_temperature"

# Extended status / diagnostic fields from the long 0x66 frame
CONF_INDOOR_HUMIDITY_SETTING = "indoor_humidity_setting"
CONF_INDOOR_HUMIDITY = "indoor_humidity"
CONF_TEMPERATURE_COMPENSATION = "temperature_compensation"
CONF_ON_TIMER_HOUR = "on_timer_hour"
CONF_ON_TIMER_MINUTE = "on_timer_minute"
CONF_ON_TIMER_ACTIVE = "on_timer_active"
CONF_OFF_TIMER_HOUR = "off_timer_hour"
CONF_OFF_TIMER_MINUTE = "off_timer_minute"
CONF_OFF_TIMER_ACTIVE = "off_timer_active"
CONF_ON_TIMER_TEXT = "on_timer_text"
CONF_OFF_TIMER_TEXT = "off_timer_text"
CONF_IAB = "iab"
CONF_IBC = "ibc"
CONF_IUV = "iuv"
CONF_COMMAND_RECEIVED_CODE = "command_received_code"
CONF_INDOOR_EEPROM = "indoor_eeprom"
CONF_COMMAND_CONFIRMATION = "command_confirmation"
CONF_OUTDOOR_RAW_47 = "outdoor_raw_47"
CONF_OUTDOOR_RAW_48 = "outdoor_raw_48"
CONF_OUTDOOR_RAW_49 = "outdoor_raw_49"
CONF_OUTDOOR_DC_BUS_RAW = "outdoor_dc_bus_raw"
CONF_OUTDOOR_TELEMETRY_71 = "outdoor_telemetry_71"
CONF_OUTDOOR_IBC_ECHO = "outdoor_ibc_echo"
CONF_OUTDOOR_IAB_ECHO = "outdoor_iab_echo"

# New memory diagnostics sensor keys
CONF_HEAP_FREE = "heap_free"
CONF_HEAP_TOTAL = "heap_total"
CONF_HEAP_USED = "heap_used"
CONF_HEAP_MIN_FREE = "heap_min_free"
CONF_HEAP_MAX_ALLOC = "heap_max_alloc"
CONF_HEAP_FRAGMENTATION = "heap_fragmentation"
CONF_PSRAM_TOTAL = "psram_total"
CONF_PSRAM_FREE = "psram_free"

CONFIG_SCHEMA = BASE_CLIMATE_SCHEMA.extend({
    **BASE_CLIMATE_EXTRA,
    cv.Optional(CONF_ENABLE_PRESETS, default=True): cv.boolean,
    cv.Optional(CONF_IR_TRANSMITTER_ID): cv.use_id(remote_base.RemoteTransmitterBase),
    cv.Optional(CONF_IFEEL_MQTT_TOPIC): cv.string_strict,
    cv.Optional(CONF_IFEEL_MQTT_PAYLOAD, default="hex"): cv.one_of("hex", "json", lower=True),
    cv.Optional(CONF_IFEEL_MQTT_QOS, default=0): cv.one_of(0, 1, 2, int=True),
    cv.Optional(CONF_IFEEL_MQTT_RETAIN, default=False): cv.boolean,

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
    cv.Optional(CONF_COMP_FR_ACTUAL): sensor.sensor_schema(),
    cv.Optional(CONF_COMP_FR_SET): sensor.sensor_schema(),
    cv.Optional(CONF_COMP_FR_COMMAND): sensor.sensor_schema(),
    cv.Optional(CONF_COMP_FR): sensor.sensor_schema(),
    cv.Optional(CONF_OUTDOOR_TEMP): sensor.sensor_schema(),
    cv.Optional(CONF_OUTDOOR_COND_TEMP): sensor.sensor_schema(),
    cv.Optional(CONF_COMPRESSOR_EXHAUST_TEMP): sensor.sensor_schema(),

    # Extended 0x66 diagnostics. These are created by default so the fields can
    # be evaluated on the real indoor/outdoor unit without additional YAML.
    cv.Optional(CONF_INDOOR_HUMIDITY_SETTING, default={CONF_NAME: "Indoor Humidity Setting"}): sensor.sensor_schema(unit_of_measurement="%", device_class="humidity", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_INDOOR_HUMIDITY, default={CONF_NAME: "Indoor Humidity"}): sensor.sensor_schema(unit_of_measurement="%", device_class="humidity", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_TEMPERATURE_COMPENSATION, default={CONF_NAME: "Temperature Compensation Code"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_ON_TIMER_HOUR, default={CONF_NAME: "Power-on Timer Hours"}): sensor.sensor_schema(unit_of_measurement="h", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_ON_TIMER_MINUTE, default={CONF_NAME: "Power-on Timer Minutes"}): sensor.sensor_schema(unit_of_measurement="min", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_ON_TIMER_ACTIVE, default={CONF_NAME: "Power-on Timer Active"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OFF_TIMER_HOUR, default={CONF_NAME: "Power-off Timer Hours"}): sensor.sensor_schema(unit_of_measurement="h", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OFF_TIMER_MINUTE, default={CONF_NAME: "Power-off Timer Minutes"}): sensor.sensor_schema(unit_of_measurement="min", accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OFF_TIMER_ACTIVE, default={CONF_NAME: "Power-off Timer Active"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_IAB, default={CONF_NAME: "Current IAB Raw"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_IBC, default={CONF_NAME: "Current IBC Raw"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_IUV, default={CONF_NAME: "Voltage IUV Raw"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_COMMAND_RECEIVED_CODE, default={CONF_NAME: "Command Received Code"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_INDOOR_EEPROM, default={CONF_NAME: "Indoor EEPROM Flag"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OUTDOOR_RAW_47, default={CONF_NAME: "Outdoor Unit Raw Byte 47"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OUTDOOR_RAW_48, default={CONF_NAME: "Outdoor Unit Raw Byte 48"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OUTDOOR_RAW_49, default={CONF_NAME: "Outdoor Unit Raw Byte 49"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OUTDOOR_DC_BUS_RAW, default={CONF_NAME: "Outdoor Unit DC Bus Raw"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OUTDOOR_TELEMETRY_71, default={CONF_NAME: "Outdoor Unit Telemetry 71 Raw"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OUTDOOR_IBC_ECHO, default={CONF_NAME: "Outdoor IBC Echo Raw"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OUTDOOR_IAB_ECHO, default={CONF_NAME: "Outdoor IAB Echo Raw"}): sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),

    cv.Optional(CONF_ON_TIMER_TEXT, default={CONF_NAME: "Power-on Timer"}): text_sensor.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_OFF_TIMER_TEXT, default={CONF_NAME: "Power-off Timer"}): text_sensor.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    cv.Optional(CONF_COMMAND_CONFIRMATION, default={CONF_NAME: "Command Confirmation"}): text_sensor.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),

    # Power status is a text sensor ("ON"/"OFF")
    cv.Optional(CONF_POWER_STATUS): text_sensor.text_sensor_schema(),

    # Existing optional sensor and LED switch
    cv.Optional(CONF_PIPE_TEMPERATURE): sensor.sensor_schema(),
    cv.Optional(CONF_LED_SWITCH): switch.switch_schema(
        ACHILEDTargetSwitch,
        icon=ICON_LIGHTBULB,
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
    # Command sound switch is created by default. The user may still override its
    # name/icon by specifying sound_switch: in YAML.
    cv.Optional(CONF_SOUND_SWITCH, default={CONF_NAME: "AC Command Sound"}): switch.switch_schema(
        ACHICommandSoundSwitch,
        icon="mdi:volume-high",
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
    # Memory switch is created by default. OFF keeps the original behavior;
    # ON restores the last active HVAC mode on generic climate.turn_on.
    cv.Optional(CONF_MEMORY_SWITCH, default={CONF_NAME: "Memory"}): switch.switch_schema(
        ACHIMemorySwitch,
        icon="mdi:memory",
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
    # Selects the program used by the standard Climate Sleep preset.
    cv.Optional(CONF_SLEEP_PROGRAM, default={CONF_NAME: "Sleep Program"}): select.select_schema(
        ACHISleepProgramSelect,
        icon="mdi:sleep",
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

}).extend(uart.UART_DEVICE_SCHEMA).extend(cv.polling_component_schema("5s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_comp))

    cg.add(var.set_enable_presets(config[CONF_ENABLE_PRESETS]))

    if tx_id := config.get(CONF_IR_TRANSMITTER_ID):
        tx = await cg.get_variable(tx_id)
        cg.add(var.set_ir_transmitter(tx))

    if topic := config.get(CONF_IFEEL_MQTT_TOPIC):
        cg.add(var.set_ifeel_mqtt_topic(topic))
    cg.add(var.set_ifeel_mqtt_payload_format(config[CONF_IFEEL_MQTT_PAYLOAD]))
    cg.add(var.set_ifeel_mqtt_qos(config[CONF_IFEEL_MQTT_QOS]))
    cg.add(var.set_ifeel_mqtt_retain(config[CONF_IFEEL_MQTT_RETAIN]))

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

    if conf := config.get(CONF_COMP_FR_ACTUAL):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compr_freq_actual_sensor(sens))

    if conf := config.get(CONF_COMP_FR_SET):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compr_freq_set_sensor(sens))

    if conf := config.get(CONF_COMP_FR_COMMAND):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compr_freq_command_sensor(sens))

    # Legacy key retained for compatibility. It still publishes status byte 43.
    if conf := config.get(CONF_COMP_FR):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compr_freq_sensor(sens))

    if conf := config.get(CONF_OUTDOOR_TEMP):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_temp_sensor(sens))

    if conf := config.get(CONF_OUTDOOR_COND_TEMP):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_cond_temp_sensor(sens))

    if conf := config.get(CONF_COMPRESSOR_EXHAUST_TEMP):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_compressor_exhaust_temp_sensor(sens))

    # Extended status / diagnostic sensors
    if conf := config.get(CONF_INDOOR_HUMIDITY_SETTING):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_indoor_humidity_setting_sensor(sens))
    if conf := config.get(CONF_INDOOR_HUMIDITY):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_indoor_humidity_sensor(sens))
    if conf := config.get(CONF_TEMPERATURE_COMPENSATION):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_temperature_compensation_sensor(sens))
    if conf := config.get(CONF_ON_TIMER_HOUR):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_on_timer_hour_sensor(sens))
    if conf := config.get(CONF_ON_TIMER_MINUTE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_on_timer_minute_sensor(sens))
    if conf := config.get(CONF_ON_TIMER_ACTIVE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_on_timer_active_sensor(sens))
    if conf := config.get(CONF_OFF_TIMER_HOUR):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_off_timer_hour_sensor(sens))
    if conf := config.get(CONF_OFF_TIMER_MINUTE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_off_timer_minute_sensor(sens))
    if conf := config.get(CONF_OFF_TIMER_ACTIVE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_off_timer_active_sensor(sens))
    if conf := config.get(CONF_IAB):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_iab_sensor(sens))
    if conf := config.get(CONF_IBC):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_ibc_sensor(sens))
    if conf := config.get(CONF_IUV):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_iuv_sensor(sens))
    if conf := config.get(CONF_COMMAND_RECEIVED_CODE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_command_received_code_sensor(sens))
    if conf := config.get(CONF_INDOOR_EEPROM):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_indoor_eeprom_sensor(sens))
    if conf := config.get(CONF_OUTDOOR_RAW_47):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_raw_47_sensor(sens))
    if conf := config.get(CONF_OUTDOOR_RAW_48):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_raw_48_sensor(sens))
    if conf := config.get(CONF_OUTDOOR_RAW_49):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_raw_49_sensor(sens))
    if conf := config.get(CONF_OUTDOOR_DC_BUS_RAW):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_dc_bus_raw_sensor(sens))
    if conf := config.get(CONF_OUTDOOR_TELEMETRY_71):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_telemetry_71_sensor(sens))
    if conf := config.get(CONF_OUTDOOR_IBC_ECHO):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_ibc_echo_sensor(sens))
    if conf := config.get(CONF_OUTDOOR_IAB_ECHO):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_outdoor_iab_echo_sensor(sens))

    if conf := config.get(CONF_ON_TIMER_TEXT):
        ts = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_on_timer_text(ts))
    if conf := config.get(CONF_OFF_TIMER_TEXT):
        ts = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_off_timer_text(ts))
    if conf := config.get(CONF_COMMAND_CONFIRMATION):
        ts = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_command_confirmation_text(ts))

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

    # Optional command sound switch
    if sound_sw_conf := config.get(CONF_SOUND_SWITCH):
        sound_sw = await switch.new_switch(sound_sw_conf)
        cg.add(var.set_sound_switch(sound_sw))

    # Optional last-mode memory switch
    if memory_sw_conf := config.get(CONF_MEMORY_SWITCH):
        memory_sw = await switch.new_switch(memory_sw_conf)
        cg.add(var.set_memory_switch(memory_sw))

    if sleep_program_conf := config.get(CONF_SLEEP_PROGRAM):
        sleep_program = await select.new_select(
            sleep_program_conf,
            options=["Sleep 1", "Sleep 2", "Sleep 3", "Sleep 4"],
        )
        cg.add(var.set_sleep_program_select(sleep_program))

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

IFEEL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(ACHIClimate),
        cv.Required(CONF_TEMPERATURE): cv.templatable(cv.float_),
        cv.Optional(CONF_ENABLED, default=True): cv.templatable(cv.boolean),
    }
)


@automation.register_action(
    "ac_hi.ifeel",
    ACHIIFeelAction,
    IFEEL_SCHEMA,
    synchronous=True,
)
async def ifeel_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    template_ = await cg.templatable(config[CONF_TEMPERATURE], args, cg.float_)
    cg.add(var.set_temperature(template_))
    template_ = await cg.templatable(config[CONF_ENABLED], args, bool)
    cg.add(var.set_enabled(template_))
    return var

