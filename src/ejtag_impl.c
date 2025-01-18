
#include <tusb.h>
#include "usb_descriptors.h"

uint32_t lsejtag_impl_usbrx_len() {
    return tud_vendor_n_available(ITF_NUM_EJTAG);
}

void lsejtag_impl_usbrx_consume(uint8_t *dest, uint32_t len) {
    tud_vendor_n_read(ITF_NUM_EJTAG, dest, len);
}
