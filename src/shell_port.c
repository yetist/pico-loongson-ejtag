
#include "shell_port.h"
#include <tusb.h>

short shell_port_read_usb(char *data, unsigned short len) {
    // return tud_cdc_read(data, len);
    return 0;
}

short shell_port_write_usb(char *data, unsigned short len) {
    // if (!tud_cdc_connected()) {
    //     return len;
    // }
    uint32_t count = tud_cdc_write_available();
    if (count > len) {
        count = len;
    }
    tud_cdc_write(data, count);
    tud_cdc_write_flush();
    while (tud_cdc_write_available() != 64)
        tud_task();
    return count;
}