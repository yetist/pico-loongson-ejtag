
#pragma once

#include <stdint.h>

typedef struct {
    // PIO in/out addresses
    volatile uint32_t *tdi_write_addr;
    volatile uint32_t *tms_write_addr;
    volatile uint32_t *tdo_write_addr;
    const volatile uint32_t *tdo_read_addr;

    // WS2812
    volatile uint32_t *ws2812_write_addr;
} ejtag_impl_ctx;
