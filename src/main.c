
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <tusb.h>
#include <string.h>

#include "lsejtag/lsejtag.h"
#include "config.h"
#include "ejtag.h"

#include "ejtag.pio.h"
#include "ws2812.pio.h"

lsejtag_ctx lsejtag_lib_ctx;
ejtag_impl_ctx ejtag_ctx;

void init_hardware();
void init_software();

int main() {
    init_hardware();
    init_software();

    while (1) {
        // Fetch from FIFO prevent TDO SM from stalling
        if (!pio_sm_is_rx_fifo_empty(pio0, 2)) {
            int tdo_val = *ejtag_ctx.tdo_read_addr;
            if (tdo_val != 0x5a5a5a5a) {
                panic("TDO readout invalid (expect 0x5a5a5a5a, got 0x%08x)", tdo_val);
            }
        }

        // Stop EJTAG
        pio_set_sm_mask_enabled(pio0, 0x7, false);

        // TDI: 57 bits
        // [8+5 Bits of 0] 1-0-0-0 (IDCODE) [Zeros]
        *ejtag_ctx.tdi_write_addr = 55 - 1; // Bit count
        *ejtag_ctx.tdi_write_addr = 0x00002000;
        *ejtag_ctx.tdi_write_addr = 0x00000000;

        // TMS: 57 bits
        // 1-1-1-1-1-1-1-0 (Go to Run-Test/Idle)
        // 0-1-1-0-0 [3x 0] 1-1-1 (Update-IR) 0-0 (Shift-DR) [31x 0] 1-1-0 (Idle)
        *ejtag_ctx.tms_write_addr = 55 - 1; // Bit count
        *ejtag_ctx.tms_write_addr = 0x0007067F;
        *ejtag_ctx.tms_write_addr = 0x00300000;

        // TDI:
        // Skip (8+5+3+3+2) * 4 cycles (Garbage data)
        *ejtag_ctx.tdo_write_addr = 4 * 21 - 1; // Initial delay cycles
        *ejtag_ctx.tdo_write_addr = 32 - 1; // Read length

        // Enable EJTAG
        pio_enable_sm_mask_in_sync(pio0, 0x7);

        sleep_ms(1000);
        *ejtag_ctx.ws2812_write_addr = 0x10200000;
        sleep_ms(1000);
        *ejtag_ctx.ws2812_write_addr = 0x00200000;
    }
}

//==============================================================================
// Initialization
//==============================================================================

void init_gpio() {
    gpio_init(EJTAG_PIN_TRST);
    gpio_init(EJTAG_PIN_BRST);
    gpio_init(EJTAG_PIN_DINT);
    
    gpio_set_dir(EJTAG_PIN_TRST, true);
    gpio_set_dir(EJTAG_PIN_BRST, true);
    gpio_set_dir(EJTAG_PIN_DINT, true);

    gpio_set_pulls(EJTAG_PIN_TRST, true, false);
    gpio_set_pulls(EJTAG_PIN_BRST, true, false);
    gpio_set_pulls(EJTAG_PIN_DINT, true, false);

    gpio_put(EJTAG_PIN_TRST, true);
    gpio_put(EJTAG_PIN_BRST, true);
    gpio_put(EJTAG_PIN_DINT, true);
}

void init_pio() {
    PIO pio;
    uint sm;
    uint offset;
    pio_sm_config smcfg = pio_get_default_sm_config();
    static const char panic_text[] = "%s PIO SM allocation failed. This should not happen.";

    // TDI/TMS SM program
    if (pio_add_program_at_offset(pio0, &ejtag_tdi_program, 0) < 0) {
        panic(panic_text, "TDI/TMS");
    }
    if (pio_add_program_at_offset(pio0, &ejtag_tdo_program, 8) < 0) {
        panic(panic_text, "TDO");
    }
    if (pio_add_program_at_offset(pio0, &ws2812_program, 16) < 0) {
        panic(panic_text, "WS2812");
    }

    // EJTAG GPIO alternative function selection
    pio_gpio_init(pio0, EJTAG_PIN_TDI);
    pio_gpio_init(pio0, EJTAG_PIN_TDO);
    pio_gpio_init(pio0, EJTAG_PIN_TCK);
    pio_gpio_init(pio0, EJTAG_PIN_TMS);
    pio_gpio_init(pio0, EJTAG_PIN_TDO_SAMPLE_TIME);

    // TDI SM (PIO0, SM0) config
    pio_sm_set_consecutive_pindirs(pio0, 0, EJTAG_PIN_TDI, 1, true);
    pio_sm_set_consecutive_pindirs(pio0, 0, EJTAG_PIN_TCK, 1, true);
    sm_config_set_fifo_join(&smcfg, PIO_FIFO_JOIN_NONE);
    sm_config_set_out_pins(&smcfg, EJTAG_PIN_TDI, 1);
    sm_config_set_out_shift(&smcfg, true, false, 0);
    sm_config_set_sideset_pins(&smcfg, EJTAG_PIN_TCK);
    sm_config_set_sideset(&smcfg, 1, false, false);
    sm_config_set_wrap(&smcfg, ejtag_tdi_wrap_target, ejtag_tdi_wrap);
    sm_config_set_clkdiv(&smcfg, 32.f);
    pio_sm_init(pio0, 0, 0, &smcfg);
    ejtag_ctx.tdi_write_addr = &pio0->txf[0];

    // TMS SM (PIO0, SM1) config
    pio_sm_set_consecutive_pindirs(pio0, 1, EJTAG_PIN_TMS, 1, true);
    sm_config_set_out_pins(&smcfg, EJTAG_PIN_TMS, 1);
    pio_sm_init(pio0, 1, 0, &smcfg);
    ejtag_ctx.tms_write_addr = &pio0->txf[1];

    // TDO SM (PIO0, SM2) config
    pio_sm_set_consecutive_pindirs(pio0, 2, EJTAG_PIN_TDO, 1, false);
    pio_sm_set_consecutive_pindirs(pio0, 2, EJTAG_PIN_TDO_SAMPLE_TIME, 1, true);
    sm_config_set_sideset_pins(&smcfg, EJTAG_PIN_TDO_SAMPLE_TIME);
    sm_config_set_in_pins(&smcfg, EJTAG_PIN_TDO);
    sm_config_set_wrap(&smcfg, ejtag_tdo_wrap_target + 8, ejtag_tdo_wrap + 8);
    pio_sm_init(pio0, 2, 8, &smcfg);
    ejtag_ctx.tdo_write_addr = &pio0->txf[2];
    ejtag_ctx.tdo_read_addr = &pio0->rxf[2];

    pio_sm_clear_fifos(pio0, 2);

    // WS2812 SM (PIO0, SM3) config
    ws2812_program_init(pio0, 3, 16, WS2812_PIN, 800000, false);
    ejtag_ctx.ws2812_write_addr = &pio0->txf[3];
}

void init_hardware() {
    init_gpio();
    init_pio();
}


void init_software() {
    tusb_init();
}