
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "cmd_headers.h"

// INTERNAL USE.
typedef enum {
    op_probe_mem_rw                     = 0x01,
    op_io_manip                         = 0x03,
    op_ir_rw                            = 0x04,
    op_dr_rw                            = 0x05,
    op_loopback_test                    = 0x08,
    op_fast_target_mem_write_fastdata   = 0x0c,
    op_fast_target_mem_write            = 0x0d,
    op_fast_target_mem_read_fastdata    = 0x0e,
    op_fast_target_mem_read             = 0x0f,
    op_get_firmware_date                = 0x1f,
} lsejtag_cmd_op;

// INTERNAL USE.
typedef struct {
    union {
        uint16_t u16;
        lsejtag_cmd_header_common common;
        lsejtag_cmd_header_probe_mem_rw probe_mem_rw;
        lsejtag_cmd_header_io_manip io_manip;
        lsejtag_cmd_header_ir_rw ir_rw;
        lsejtag_cmd_header_dr_rw dr_rw;
        lsejtag_cmd_header_loopback_test loopback_test;
        lsejtag_cmd_header_fast_target_mem_write fast_target_mem_write;
        lsejtag_cmd_header_fast_target_mem_read fast_target_mem_read;
    };
} lsejtag_cmd_header;

// INTERNAL USE.
typedef struct {
    union {
        lsejtag_cmd_common common;
        lsejtag_cmd_probe_mem_rw probe_mem_rw;
        lsejtag_cmd_io_manip io_manip;
        lsejtag_cmd_ir_rw ir_rw;
        lsejtag_cmd_dr_rw dr_rw;
        lsejtag_cmd_loopback_test loopback_test;
        lsejtag_cmd_fast_target_mem_write_at_core fast_mem_write_at_core;
        lsejtag_cmd_fast_target_mem_write fast_mem_write;
        lsejtag_cmd_fast_target_mem_read_at_core fast_mem_read_at_core;
    };
} lsejtag_cmd;

// INTERNAL USE. Command buffering state machine state.
typedef enum {
    cbs_clean,          // The buffer is clean, no pending commands
    cbs_wait_header,    // Pending command has only 1 byte available, command header is incomplete
    cbs_wait_cfg,       // Pending command's header is available, but config section is incomplete
    cbs_wait_payload,   // Pending command's header and config section is available, but payload
                        // is incomplete. In this case, `cmd_buf_wait_payload_size` represents how
                        // many bytes of payload data is yet to be read.
    cbs_execute,        // The buffer contains a valid command and awaits execution.
} lsejtag_cmd_buf_state;

typedef enum {
    fcr_execute,        // A command was successfully fetched, call `lsejtag_exec_cmd` immediately.
    fcr_clean,          // Command buffer and user provided USB RX FIFO are both empty. This result
                        // indicates a clean end state.
    fcr_incomplete,     // An incomplete command was fetched and buffered. User USB RX FIFO is
                        // exhausted.
    fcr_flush_tdo,      // User should call `lsejtag_flush_tdo` immediately to flush TDO. This code
                        // does not imply any status of command buffer; `lsejtag_fetch_cmd` must be
                        // called again immediately after flushing TDO buffer to find out current
                        // status of command buffer.
    fcr_too_long,       // An excessively long command was found and was too large to fit in scratch
                        // buffer. This is a fatal error. Notice that this error is not emitted for
                        // `op_fast_target_mem_write*` ops, since this op really do deal with large
                        // amount of data. Special continuation logic is implemented for this op
                        // to make it work across multiple `lsejtag_execute` calls.
    fcr_unknown_cmd,    // An unknown command was found. This may be emitted after command header
                        // was read. Ignore this error.
    fcr_corrupt_state,  // An invalid cmd_buf_state was found. This is a fatal error.
} lsejtag_fetch_cmd_result;

typedef enum {
    cer_success,        // Command executed successfully and all peripheral accesses have finished.
                        // This is returned for commands that can finish without manipulating JTAG
                        // port, for example, firmware version request.
    cer_jtag_started,   // Command executed successfully, JTAG port is doing data exchange
    cer_continue,       // Command was partially executed, because more data has to be sent to/
                        // received from the host. This is a special return code for
                        // fast_target_mem_read/write ops. When this code was returned, buffer state
                        // is not cleared to cbs_clean, and command continuation context info will
                        // be filled in preparation to next stage of execution.
    cer_await_jtag_buf, //
    cer_await_usb_tx,   // Command was not executed because USB TX FIFO is full.
    cer_not_ready,      // Command was not executed because command buffer state is not ready.
    cer_bad_continue,   // Continuation context was valid for a command that does not support
                        // continuation. This is a fatal error.
} lsejtag_cmd_exec_result;

typedef struct {
    // This command buffer is internally used as both a scratch area and a command buffer. Any
    // command sequence read by LS-EJTAG will be first written to this buffer before it's contents
    // were processed. That was the scratch area case. When a long command was split by USB
    // transaction boundary, the full content of that command might not be available immediately.
    // LS-EJTAG will buffer what is immediately available into this buffer first, and when later
    // USB packets arrived it will assemble the full content of the command in this buffer and send
    // it to execution.
    // Depending on how many bytes of a command is available at first, the cmd_buf_state will be
    // changed so that proper read strategy can be decided when next packet was received.
    uint32_t buffered_length;
    uint8_t buffered_cmd[256];
    uint32_t cmd_buf_wait_payload_size;
    lsejtag_cmd_buf_state cmd_buf_state;

    // Configurable parameters. These are mostly accessed by `usblooptest` commands by "writing
    // directly into probe memory space".
    uint8_t param_ir_len_bits;
    uint8_t param_ir_skip;
    uint8_t param_ir_control;
    uint8_t param_ir_data;
    uint8_t param_ir_address;
    uint8_t param_ir_fastdata;
    uint16_t param_jtag_clk_div;
    uint8_t param_jtag_tdo_sample_timing;


    // Command continuation context. fast_target_mem_read/write ops may require receiving/sending
    // more data than it can receive/send in one go, therefore they have to be able to save the
    // partially executed state, and continue later on when more data can be received/sent.
    bool continuation;  // Valid flag
    uint8_t cont_op;    // Which op was this continuation context created for
    uint32_t dword_left;// How many DWORDs to read/write before command finishes.
                        // For fast_writes: how many DWORDs of payload need to be received from host
                        // For fast_reads: how many DWORDs of RAM need to be read from target (and
                        // encapsulated in the format that host understands and sent)
    
    struct {
        // JTAG IO busy indicators. This is used by `lsejtag_execute` to determine whether a command
        // can execute (if JTAG peripheral is still working and the current command has something to
        // do with JTAG port, then execution should be delayed). All flags are set when DMA was
        // started. TDI and TMS are cleared by DMA finish ISRs. TDO is cleared by
        // `lsejtag_flush_tdo()` after tdo_data buffer is written to USB TX FIFO.
        union {
            struct {
                bool tdi_busy;
                bool tdo_busy;
                bool tms_busy;
            };
            uint32_t busy;
        };
        // TDO buffer. tdo_data is the DMA-reader destination buffer. When DMA is finished,
        // tdo_dwords should be filled by DMA ISR.
        uint32_t tdo_dwords;    // How many DWORDs of collected TDO data needs to be sent through
                                // USB. Only valid when larger than 0, and will be detected by
                                // `lsejtag_fetch_cmd()` when it becomes larger than 0.
        uint32_t tdo_data[64];
        // TDI/TMS buffer.
        uint32_t tdi_data[64];
        uint32_t tms_data[64];
        uint32_t tdi_bits;      // How many bits should be sent to TDI and TMS.
    } jtagbuf_a, jtagbuf_b;
} lsejtag_ctx;

//==============================================================================
// API

void lsejtag_init_ctx(lsejtag_ctx *ctx);

lsejtag_fetch_cmd_result lsejtag_fetch_cmd(lsejtag_ctx *ctx);


