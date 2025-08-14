import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

# Нужны для подтягивания заголовков C++
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "switch", "text_sensor", "number", "select"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")
ACHIComponent = ac_hi_ns.class_("ACHIComponent", cg.PollingComponent, uart.UARTDevice)

CONF_NAME_PREFIX = "name_prefix"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ACHIComponent),
            cv.Optional(CONF_NAME_PREFIX, default="Hisense AC"): cv.string,
        }
    )
    .extend(cv.polling_component_schema("1s"))   # интервал тиков FSM
    .extend(uart.UART_DEVICE_SCHEMA)             # параметры UART
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_name_prefix(config[CONF_NAME_PREFIX]))
