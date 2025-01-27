
#pragma once

#include <stdint.h>
#include "lsejtag/lsejtag.h"

typedef struct {
    // PIO in/out addresses
    volatile uint32_t *tdi_write_addr;
    volatile uint32_t *tms_write_addr;
    volatile uint32_t *tdo_write_addr;
    const volatile uint32_t *tdo_read_addr;

    // This is provided to library when a readout completed
    uint32_t tdo_recv_dword_count;

    // DMA channel
    int tdi_chan;
    int tms_chan;
    int tdo_chan;
    uint32_t dma_start_mask;
    uint32_t dma_start_mask_no_tdo;

    // WS2812
    volatile uint32_t *ws2812_write_addr;

    // LED status "switch", each time this flag is flipped to 1, the LED task will keep LED
    // illuminated for 300ms, and clear this flag. The next field is the time-keeper field for LED
    // task. When it is non-zero it is checked each time LED task is called to determine when to
    // turn LED off. When it is zero it means LED is already turned off.
    uint32_t led_turn_on;
    uint32_t led_timer_us;
} ejtag_impl_ctx;

// User implementation side context (in main.c)
extern ejtag_impl_ctx ejtag_ctx;

// LS-EJTAG library context (in main.c)
extern lsejtag_ctx lsejtag_lib_ctx;

// DMA IRQ Handler
void isr_dma_irq0();
void isr_dma_irq1();
// PIO IRQ Handler
void isr_pio0_irq0();
