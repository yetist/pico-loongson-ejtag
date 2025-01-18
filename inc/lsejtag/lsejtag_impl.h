
#pragma once

#include <stdint.h>


/*==============================================================================
 *                       P O R T I N G    G U I D E
 *==============================================================================
 *
 * 1. Functions to implement
 *   
 *  uint32_t lsejtag_impl_usbrx_len();
 *      Should return how many bytes has been received from USB and can be taken
 *      from USB RX FIFO immediately by calling lsejtag_impl_usbrx_consume.
 *
 *  uint32_t lsejtag_impl_usbtx_free_len();
 *      Should return how many bytes can be put into USB TX FIFO immediately and
 *      sent out to host by calling lsejtag_impl_usbtx.
 *
 *  void lsejtag_impl_usbrx_consume(uint8_t *dest, uint32_t len);
 *      Should consume `len` bytes from your USB RX FIFO, and write them to
 *      buffer pointed to by `dest`. `len` will never be larger than the return
 *      value of `lsejtag_impl_usbrx_len()`, which will always be called before
 *      `lsejtag_impl_usbrx_consume()` was called.
 * 
 *  void lsejtag_impl_usbtx(uint8_t *data, uint32_t len);
 *      Should add `len` bytes into your USB TX FIFO. The sending part may be
 *      done later after `lsejtag_execute()` has returned. Each call to
 *      `lsejtag_execute()` will result in only one `lsejtag_impl_usbtx()` call.
 *      Sending of asynchronously collected TDO data is handled inside
 *      `lsejtag_flush_tdo()` when `lsejtag_fetch_cmd()` asks you to do so.
 * 
 *  void lsejtag_impl_io_manip(lsejtag_impl_io io, uint8_t level);
 *      TODO:
 *
 *============================================================================*/

typedef enum {
    io_led,                 // Green LED. Default to low.
    io_fpga_reset,          // Non-used.
    io_oe,                  // JTAG output enable. When OE is low output is enabled. Default to low.
    io_trst,                // TRST pin. Default to high.
    io_brst,                // BRST pin. Default to high.
    io_dint,                // DINT pin. Default to high.
    io_tap_reset,           // Non-used.
} lsejtag_impl_io;

uint32_t lsejtag_impl_usbrx_len();

uint32_t lsejtag_impl_usbtx_free_len();

void lsejtag_impl_usbrx_consume(uint8_t *dest, uint32_t len);

void lsejtag_impl_usbtx(const uint8_t *data, uint32_t len);

void lsejtag_impl_io_manip(lsejtag_impl_io io, uint8_t level);
