# SPDX-License-Identifier: MIT
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_UPDATE_INTERVAL,
)

AUTO_LOAD = ["uart", "climate"]
CODEOWNERS = ["@artshevchenko"]

ac_hi_ns = cg.esphome_ns.namespace("ac_hi")
ACHiClimate = ac_hi_ns.class_("ACHiClimate", climate.Climate, cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.All(
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(ACHiClimate),
            cv.Optional(CONF_NAME, default="Hisense/Ballu AC"): cv.string,
            cv.Optional(CONF_UPDATE_INTERVAL, default="2s"): cv.update_interval,
        }
    ).extend(uart.UART_DEVICE_SCHEMA),
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await climate.register_climate(var, config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_status_update_interval(uint32_t(config[CONF_UPDATE_INTERVAL].total_milliseconds)))
