"""Virtual UART <-> RF433 bridge (Nartis).

This component PRESENTS a UART (`uart::UARTComponent`). Any component that talks
to a meter "over UART" can bind to it - by pointing its `uart_id` at this
component - and its requests/replies are transparently relayed over a 433 MHz
radio. Requests written to the virtual UART are collected (end detected by an
idle gap or an upstream flush()), packed into an RF packet and transmitted; the
bridge then listens on RF for a reply, unpacks it and serves it back through the
read side of the virtual UART.

The CMT2300A radio driver (bit-bang SPI HAL), the type-5A DLMS-HDLC envelope
(build on TX, CRC-carve + unwrap on RX), link-layer ARQ, and the non-blocking
bridge state machine are all implemented. The frequency is fixed at 443.9 MHz for
now (serial-derived selection is a TODO).
"""

from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID
from esphome import automation

CODEOWNERS = ["@latonita"]
# We PROVIDE a uart::UARTComponent; other components consume it via uart_id. The
# ring_buffer holds replies received over RF until the upstream reads them.
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["uart", "ring_buffer"]
MULTI_CONF = True

# --- Configuration keys -----------------------------------------------------
# CMT2300A wiring (bit-bang 3-wire SPI + INT).
CONF_PIN_SDIO = "pin_sdio"
CONF_PIN_SCLK = "pin_sclk"
CONF_PIN_CSB = "pin_csb"
CONF_PIN_FCSB = "pin_fcsb"
CONF_PIN_GPIO3 = "pin_gpio3"

CONF_ADDRESS = "address"

CONF_REQUEST_GAP = "request_gap"
CONF_RF_TX_TIMEOUT = "rf_tx_timeout"
CONF_RF_RX_TIMEOUT = "rf_rx_timeout"
CONF_RF_RETRIES = "rf_retries"
CONF_RX_CENTER_OFFSET = "rx_center_offset"

CONF_ON_UART_MESSAGE = "on_uart_message"
CONF_ON_RF_REPLY = "on_rf_reply"
CONF_ON_RF_TIMEOUT = "on_rf_timeout"

uart_nartis_rf_ns = cg.esphome_ns.namespace("uart_nartis_rf")
UartNartisRfComponent = uart_nartis_rf_ns.class_(
    "UartNartisRfComponent", uart.UARTComponent, cg.Component
)


def validate_address(value):
    """Meter serial / RF address: the 12-digit number printed on the meter."""
    s = cv.string_strict(value)
    if not s.isdigit() or len(s) != 12:
        raise cv.Invalid(
            f"address must be exactly 12 digits (the meter serial, e.g. "
            f"'023240271060'); got {len(s)} characters '{s}'"
        )
    return s

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UartNartisRfComponent),
        # CMT2300A wiring (bit-bang 3-wire SPI + INT).
        cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_GPIO3): pins.internal_gpio_input_pin_schema,
        # Meter serial / RF address (12 digits). Goes into the RF packet and its
        # last 3 digits select the operating frequency.
        cv.Required(CONF_ADDRESS): validate_address,
        # End-of-request idle gap on the write side: once the upstream stops
        # writing for this long (or calls flush()), the bytes collected so far
        # are treated as one complete request and relayed over RF.
        cv.Optional(
            CONF_REQUEST_GAP, default="100ms"
        ): cv.positive_time_period_milliseconds,
        # How long to wait for the radio to confirm the packet left the antenna
        # before giving up on this transmission.
        cv.Optional(
            CONF_RF_TX_TIMEOUT, default="1000ms"
        ): cv.positive_time_period_milliseconds,
        # How long to stay in RF RX waiting for a reply before declaring a timeout
        # and returning to idle.
        cv.Optional(
            CONF_RF_RX_TIMEOUT, default="1000ms"
        ): cv.positive_time_period_milliseconds,
        # RF-layer ARQ: retransmissions on no-reply / bad CRC. 0 = pure transparent
        # (no retry). Keep (rf_retries + 1) * rf_rx_timeout below the upstream's
        # receive timeout so its own retry never fires while ARQ is still working.
        cv.Optional(CONF_RF_RETRIES, default=2): cv.int_range(min=0, max=10),
        # RX center offset in CMT2300A frequency codes (1 code ~= 6.199 Hz). Shifts
        # the RX-half LO onto the meter's reply carrier (~+4.7 kHz above our TX).
        # Default +758 = the value proven in the test app. Tune per install if the
        # reply sits elsewhere; the wide RX profile + AFC absorb normal drift.
        cv.Optional(CONF_RX_CENTER_OFFSET, default=758): cv.int_range(min=-4000, max=4000),
        cv.Optional(CONF_ON_UART_MESSAGE): automation.validate_automation({}),
        cv.Optional(CONF_ON_RF_REPLY): automation.validate_automation({}),
        cv.Optional(CONF_ON_RF_TIMEOUT): automation.validate_automation({}),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    sdio = await cg.gpio_pin_expression(config[CONF_PIN_SDIO])
    cg.add(var.set_pin_sdio(sdio))
    sclk = await cg.gpio_pin_expression(config[CONF_PIN_SCLK])
    cg.add(var.set_pin_sclk(sclk))
    csb = await cg.gpio_pin_expression(config[CONF_PIN_CSB])
    cg.add(var.set_pin_csb(csb))
    fcsb = await cg.gpio_pin_expression(config[CONF_PIN_FCSB])
    cg.add(var.set_pin_fcsb(fcsb))
    # Interrupt pin: the chip's GPIO3 pad (INT2). The HAL polls this for the
    # FIFO-threshold signal that drives RX/TX FIFO draining.
    gpio3 = await cg.gpio_pin_expression(config[CONF_PIN_GPIO3])
    cg.add(var.set_pin_gpio3(gpio3))

    cg.add(var.set_address(config[CONF_ADDRESS]))

    cg.add(var.set_request_gap_ms(config[CONF_REQUEST_GAP]))
    cg.add(var.set_rf_tx_timeout_ms(config[CONF_RF_TX_TIMEOUT]))
    cg.add(var.set_rf_rx_timeout_ms(config[CONF_RF_RX_TIMEOUT]))
    cg.add(var.set_rf_retries(config[CONF_RF_RETRIES]))
    cg.add(var.set_rx_center_offset(config[CONF_RX_CENTER_OFFSET]))

    for conf in config.get(CONF_ON_UART_MESSAGE, []):
        await automation.build_callback_automation(
            var, "add_on_uart_message_callback", [], conf
        )
    for conf in config.get(CONF_ON_RF_REPLY, []):
        await automation.build_callback_automation(
            var, "add_on_rf_reply_callback", [], conf
        )
    for conf in config.get(CONF_ON_RF_TIMEOUT, []):
        await automation.build_callback_automation(
            var, "add_on_rf_timeout_callback", [], conf
        )
