//
// Created by vidma on 28/09/2025.
//


#include "SimpleConsoleDriver.h"
#include <AP_Common/ExpandingString.h>
#include <stdio.h>

ESP32::UARTDriver::UARTDriver() {}

/* Empty implementations of virtual methods */
void ESP32::UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void ESP32::UARTDriver::_end() {}
void ESP32::UARTDriver::_flush() {}
bool ESP32::UARTDriver::is_initialized() { return false; }
bool ESP32::UARTDriver::tx_pending() { return false; }

uint32_t ESP32::UARTDriver::_available() { return 0; }
uint32_t ESP32::UARTDriver::txspace() { return 1; }
bool ESP32::UARTDriver::_discard_input() { return false; }
size_t ESP32::UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    // dummy console impl using stdio from pico-sdk
    for (size_t i=0; i<size; i++) {
        putchar(buffer[i]);
    }
    return size;
}
ssize_t ESP32::UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    return 0;
}

#if HAL_UART_STATS_ENABLED
void ESP32::UARTDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
{
    str.printf("EMPTY\n");
}
#endif
