
#pragma once

#include <stdint.h>
#include <stdbool.h>

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
 *      `lsejtag_flush_tdo()` when `lsejtag_dispatch()` asks you to do so.
 * 
 *  void lsejtag_impl_io_manip(lsejtag_impl_io io, uint8_t level);
 *      TODO:
 *
 *  void lsejtag_impl_run_jtag_setup(uint32_t tdi_bits, uint32_t tdo_bits,
 *                                   uint32_t tdo_skip_bits)
 *      TODO:
 *
 *============================================================================*/

typedef enum {
    impl_io_led,
    impl_io_trst,
    impl_io_brst,
    impl_io_dint,
} lsejtag_impl_io;

typedef enum {
    impl_recfg_clkfreq,
    impl_recfg_tdo_sample_time,
} lsejtag_impl_recfg;

uint32_t lsejtag_impl_usbrx_len();

uint32_t lsejtag_impl_usbtx_free_len();

void lsejtag_impl_usbrx_consume(uint8_t *dest, uint32_t len);

void lsejtag_impl_usbtx(const uint8_t *data, uint32_t len);

void lsejtag_impl_io_manip(lsejtag_impl_io io, uint8_t level);

void lsejtag_impl_reconfigure(lsejtag_impl_recfg type, uint32_t param);

void lsejtag_impl_run_jtag(const uint32_t *tdi_buf, const uint32_t *tms_buf, uint32_t *tdo_buf,
                           uint32_t tdi_bits, uint32_t tdo_bits, uint32_t tdo_skip_bits);
