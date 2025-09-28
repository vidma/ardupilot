//
// Created by vidma on 28/09/2025.
//


#include "ConsoleFakeUartDriver.h"
#include <AP_Common/ExpandingString.h>
#include <stdio.h>
#include <unistd.h>

namespace ESP32 {
    ConsoleFakeUartDriver::ConsoleFakeUartDriver() {}

    /* Empty implementations of virtual methods */
    void ConsoleFakeUartDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
    void ConsoleFakeUartDriver::_end() {}
    void ConsoleFakeUartDriver::_flush() {}
    bool ConsoleFakeUartDriver::is_initialized() { return false; }
    bool ConsoleFakeUartDriver::tx_pending() { return false; }

    uint32_t ConsoleFakeUartDriver::_available() { return 0; }
    uint32_t ConsoleFakeUartDriver::txspace() { return 1; }
    bool ConsoleFakeUartDriver::_discard_input() { return false; }
    size_t ConsoleFakeUartDriver::_write(const uint8_t *buffer, size_t size)
    {
        // dummy console impl using stdio from pico-sdk
        for (size_t i=0; i<size; i++) {
            putchar(buffer[i]);
        }
        return size;
    }
    ssize_t ConsoleFakeUartDriver::_read(uint8_t *buffer, uint16_t size)
    {
        return 0;
    }

#if HAL_UART_STATS_ENABLED
    void ConsoleFakeUartDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
    {
        str.printf("EMPTY\n");
    }
#endif
}