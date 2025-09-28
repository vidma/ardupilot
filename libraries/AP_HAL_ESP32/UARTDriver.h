//
// Created by vidma on 28/09/2025.
//

#pragma once

//#include "AP_HAL_Empty.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>

#include <hardware/uart.h>

namespace ESP32 {
    class UARTDriver : public AP_HAL::UARTDriver {
    public:
        UARTDriver(int uart_num);
        /* Empty implementations of UARTDriver virtual methods */
        bool is_initialized() override;
        bool tx_pending() override;

        /* Empty implementations of Stream virtual methods */
        uint32_t txspace() override;


#if HAL_UART_STATS_ENABLED
        // request information on uart I/O for one uart
        void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) override;
#endif

    protected:
        uart_inst_t *uart_inst;
        bool _initialized;
        int _uart_num;

        void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
        size_t _write(const uint8_t *buffer, size_t size) override;
        ssize_t _read(uint8_t *buffer, uint16_t size) override WARN_IF_UNUSED;
        void _end() override;
        void _flush() override;
        uint32_t _available() override;
        bool _discard_input() override;
    };
}
