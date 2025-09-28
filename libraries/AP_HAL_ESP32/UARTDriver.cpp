
//
// Created by vidma on 28/09/2025.
//


#include "UARTDriver.h"
#include <AP_Common/ExpandingString.h>
#include <stdio.h>
#include <unistd.h>

#include <hardware/gpio.h>
#include <hardware/uart.h>

// FIXME: only blocking impl for now
namespace ESP32 {
    // https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_uart
    UARTDriver::UARTDriver(int uart_num) {
        _initialized = false;
        _uart_num = uart_num;

        // e.g.: Initialise UART 0
        // Set the GPIO pin mux to the UART - pin 0 is TX, 1 is RX; note use of UART_FUNCSEL_NUM for the general
        // case where the func sel used for UART depends on the pin number
        // Do this before calling uart_init to avoid losing data
        // FIXME: make pin numbers configurable
        if (_uart_num == 0) {
            uart_inst = uart0;
            gpio_set_function(0, UART_FUNCSEL_NUM(uart_inst, 0));
            gpio_set_function(1, UART_FUNCSEL_NUM(uart_inst, 1));
        }
        if (_uart_num == 1) {
            uart_inst = uart1;
            gpio_set_function(4, UART_FUNCSEL_NUM(uart_inst, 4));
            gpio_set_function(5, UART_FUNCSEL_NUM(uart_inst, 5));
        }
        uart_init(uart_inst, 115200);
        _initialized = true;
    }

    void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {
        // FIXME - this one is probably not always called!!!
        // if initialization was here, nothing is printed!
    }
    void UARTDriver::_end() {}
    void UARTDriver::_flush() {}
    bool UARTDriver::is_initialized() { return _initialized; }
    bool UARTDriver::tx_pending() { return false; }

    uint32_t UARTDriver::_available() {
        if (!_initialized) {
            return 0;
        }
        // FIXME: readbuf
        return (uart_is_readable(uart_inst)) ? 1 : 0;
        // about IRQs here
        // https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_uart_1ga1908247cb5f2468517b37d5a91798181
    }
    uint32_t UARTDriver::txspace() { return 1; }
    bool UARTDriver::_discard_input() { return false; }
    size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
    {
        for (size_t i=0; i<size; i++) {
            uart_putc(uart_inst, buffer[i]);
        }
        return size;
    }

    ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t size)
    {
        uart_read_blocking(uart_inst, buffer, size);
        return size;
    }

#if HAL_UART_STATS_ENABLED
    void UARTDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
    {
        str.printf("EMPTY\n");
    }
#endif
}
