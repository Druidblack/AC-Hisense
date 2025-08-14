import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import uart, sensor, switch, text_sensor, number, select
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "switch", "text_sensor", "number", "select"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")

ACHIComponent = ac_hi_ns.class_("ACHIComponent", cg.PollingComponent, uart.UARTDevice)

ACHISwitch = ac_hi_ns.class_("ACHISwitch", switch.Switch)
ACHINumber = ac_hi_ns.class_("ACHINumber", number.Number)
ACHISelect = ac_hi_ns.class_("ACHISelect", select.Select)

ControlType = ac_hi_ns.enum("ControlType")

CONF_NAME_PREFIX = "name_prefix"
CONF_UPDATE_INTERVAL = "update_interval"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ACHIComponent),
            cv.Optional(CONF_NAME_PREFIX, default="Hisense AC"): cv.string,
            cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    name_prefix = config[CONF_NAME_PREFIX]

    # --------- Text sensor ----------
    power_text = cg.new_Pvariable(
        cg.new_id(text_sensor.TextSensor),
        text_sensor.TextSensor()
    )
    await text_sensor.register_text_sensor(power_text, {"name": f"{name_prefix} Power Status"})

    # --------- Numeric sensors ----------
    wind_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(wind_s, {"name": f"{name_prefix} Wind", "accuracy_decimals": 0})

    sleep_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(sleep_s, {"name": f"{name_prefix} Sleep", "accuracy_decimals": 0})

    mode_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(mode_s, {"name": f"{name_prefix} Mode", "accuracy_decimals": 0})

    t_set = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(
        t_set,
        {"name": f"{name_prefix} Temperature Set", "accuracy_decimals": 0,
         "unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}
    )

    t_cur = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(
        t_cur,
        {"name": f"{name_prefix} Temperature Current", "accuracy_decimals": 0,
         "unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}
    )

    t_pipe = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(
        t_pipe,
        {"name": f"{name_prefix} Pipe Temperature Current", "accuracy_decimals": 0,
         "unit_of_measurement": "°C", "device_class": "temperature", "state_class": "measurement"}
    )

    quiet_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(quiet_s, {"name": f"{name_prefix} Quiet", "accuracy_decimals": 0})

    turbo_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(turbo_s, {"name": f"{name_prefix} Turbo", "accuracy_decimals": 0})

    led_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(led_s, {"name": f"{name_prefix} LED", "accuracy_decimals": 0})

    eco_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(eco_s, {"name": f"{name_prefix} Economy", "accuracy_decimals": 0})

    lr_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(lr_s, {"name": f"{name_prefix} Left-Right", "accuracy_decimals": 0})

    ud_s = cg.new_Pvariable(cg.new_id(sensor.Sensor), sensor.Sensor())
    await sensor.register_sensor(ud_s, {"name": f"{name_prefix} Up-Down", "accuracy_decimals": 0})

    # --------- Switches ----------
    power_sw = cg.new_Pvariable(cg.new_id(ACHISwitch), ACHISwitch())
    cg.add(power_sw.set_parent(var))
    cg.add(power_sw.set_type(ControlType.CTRL_POWER))
    await switch.register_switch(power_sw, {"name": f"{name_prefix} Power"})

    quiet_sw = cg.new_Pvariable(cg.new_id(ACHISwitch), ACHISwitch())
    cg.add(quiet_sw.set_parent(var))
    cg.add(quiet_sw.set_type(ControlType.CTRL_QUIET))
    await switch.register_switch(quiet_sw, {"name": f"{name_prefix} Quiet Mode"})

    turbo_sw = cg.new_Pvariable(cg.new_id(ACHISwitch), ACHISwitch())
    cg.add(turbo_sw.set_parent(var))
    cg.add(turbo_sw.set_type(ControlType.CTRL_TURBO))
    await switch.register_switch(turbo_sw, {"name": f"{name_prefix} Turbo Mode"})

    led_sw = cg.new_Pvariable(cg.new_id(ACHISwitch), ACHISwitch())
    cg.add(led_sw.set_parent(var))
    cg.add(led_sw.set_type(ControlType.CTRL_LED))
    await switch.register_switch(led_sw, {"name": f"{name_prefix} LED"})

    eco_sw = cg.new_Pvariable(cg.new_id(ACHISwitch), ACHISwitch())
    cg.add(eco_sw.set_parent(var))
    cg.add(eco_sw.set_type(ControlType.CTRL_ECO))
    await switch.register_switch(eco_sw, {"name": f"{name_prefix} ECO Mode"})

    ud_sw = cg.new_Pvariable(cg.new_id(ACHISwitch), ACHISwitch())
    cg.add(ud_sw.set_parent(var))
    cg.add(ud_sw.set_type(ControlType.CTRL_UPDOWN))
    await switch.register_switch(ud_sw, {"name": f"{name_prefix} Up-Down Swing"})

    lr_sw = cg.new_Pvariable(cg.new_id(ACHISwitch), ACHISwitch())
    cg.add(lr_sw.set_parent(var))
    cg.add(lr_sw.set_type(ControlType.CTRL_LEFTRIGHT))
    await switch.register_switch(lr_sw, {"name": f"{name_prefix} Left-Right Swing"})

    # --------- Number (setpoint) ----------
    temp_num = cg.new_Pvariable(cg.new_id(ACHINumber), ACHINumber())
    cg.add(temp_num.set_parent(var))
    cg.add(temp_num.set_type(ControlType.CTRL_TEMP))
    await number.register_number(
        temp_num,
        {"name": f"{name_prefix} Temperature", "min_value": 18.0, "max_value": 28.0, "step": 1.0},
    )

    # --------- Selects ----------
    mode_sel = cg.new_Pvariable(cg.new_id(ACHISelect), ACHISelect())
    cg.add(mode_sel.set_parent(var))
    cg.add(mode_sel.set_type(ControlType.CTRL_MODE))
    await select.register_select(mode_sel, {"name": f"{name_prefix} Mode"})

    wind_sel = cg.new_Pvariable(cg.new_id(ACHISelect), ACHISelect())
    cg.add(wind_sel.set_parent(var))
    cg.add(wind_sel.set_type(ControlType.CTRL_WIND))
    await select.register_select(wind_sel, {"name": f"{name_prefix} Wind"})

    sleep_sel = cg.new_Pvariable(cg.new_id(ACHISelect), ACHISelect())
    cg.add(sleep_sel.set_parent(var))
    cg.add(sleep_sel.set_type(ControlType.CTRL_SLEEP))
    await select.register_select(sleep_sel, {"name": f"{name_prefix} Sleep"})

    # --------- Wire children into component ----------
    cg.add(var.set_entities(
        power_text,
        wind_s, sleep_s, mode_s,
        t_set, t_cur, t_pipe,
        quiet_s, turbo_s, led_s, eco_s, lr_s, ud_s,
        power_sw, quiet_sw, turbo_sw, led_sw, eco_sw, ud_sw, lr_sw,
        temp_num,
        mode_sel, wind_sel, sleep_sel
    ))
