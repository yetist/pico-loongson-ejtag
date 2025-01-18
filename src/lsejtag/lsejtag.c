
#include "lsejtag/lsejtag.h"
#include "lsejtag/lsejtag_impl.h"
#include "lsejtag/cmdop.h"

// Function prototypes
static inline lsejtag_cmd_exec_result lsejtag_execute_continuation(lsejtag_ctx *ctx);

// Code
#define BIT2DWORD(x) ((x) + 31 / 32)
#define CHECK_CMD_LENGTH(ctx) \
    do { \
        if ((ctx)->buffered_length + (ctx)->cmd_buf_wait_payload_size > \
            sizeof((ctx)->buffered_cmd)) { \
            return fcr_too_long; \
        } \
    } while (0); \

static inline bool is_continuation_capable_op(lsejtag_cmd_op op) {
    return op == op_fast_target_mem_read || op == op_fast_target_mem_write ||
           op == op_fast_target_mem_read_fastdata || op == op_fast_target_mem_write_fastdata;
}

static uint32_t usb_tx_len_required(lsejtag_ctx *ctx) {
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;
    
    switch (cmd->common.header.op) {
    case op_probe_mem_rw:
        return cmd->probe_mem_rw.header.is_write ? 0 : 4;
    case op_io_manip:
        return 0;
    case op_ir_rw:
        return cmd->ir_rw.header.is_write ? 0 : (BIT2DWORD(cmd->ir_rw.irseq_len_bits) * 4);
    case op_dr_rw:
        return cmd->dr_rw.header.is_write ? 0 : (BIT2DWORD(cmd->ir_rw.irseq_len_bits) * 4);
    case op_loopback_test:
    case op_get_firmware_date:
        return 4;
    case op_fast_target_mem_write_fastdata:
    case op_fast_target_mem_write:
        return 0;
    case op_fast_target_mem_read_fastdata:
    case op_fast_target_mem_read: {
        lsejtag_cmd_fast_target_mem_read_at_core *read_cmd = &cmd->fast_mem_read_at_core;
        uint32_t bytes_per_machine_word = 
            BIT2DWORD((read_cmd->header.cpu_is_64bit ? 64 : 32) +
                      read_cmd->header.chained_core_count) * 4;
        uint32_t bytes_to_send = bytes_per_machine_word * read_cmd->data_len_dword_count;
        return bytes_to_send;
    }
    default:
        return 0;
    }
}

void lsejtag_init_ctx(lsejtag_ctx *ctx) {
    ctx->buffered_length = 0;
    ctx->cmd_buf_state = cbs_clean;
}

lsejtag_fetch_cmd_result lsejtag_fetch_cmd(lsejtag_ctx *ctx) {
    uint32_t len_rx = lsejtag_impl_usbrx_len();
    uint32_t len_avail = len_rx + ctx->buffered_length;
    const lsejtag_cmd_header *header = (const lsejtag_cmd_header *)ctx->buffered_cmd;
    const lsejtag_cmd *cmd = (const lsejtag_cmd *)ctx->buffered_cmd;

    if (ctx->jtagbuf_a.tdo_dwords != 0 || ctx->jtagbuf_b.tdo_dwords != 0) {
        return fcr_flush_tdo;
    }

    if (len_rx == 0 && ctx->buffered_length == 0) {
        return fcr_clean;
    }

    if (ctx->cmd_buf_state == cbs_clean) {
        // Starting from a clean buffer
        if (len_rx == 0) {
            return fcr_clean;
        } else if (len_rx == 1) {
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd, 1);
            ctx->buffered_length = 1;
            ctx->cmd_buf_state = cbs_wait_header;
            return fcr_incomplete;
        }

        // Get a command header
        lsejtag_impl_usbrx_consume(ctx->buffered_cmd, 2);
        len_rx -= 2;
        len_avail -= 2;
        ctx->buffered_length = 2;
        ctx->cmd_buf_state = cbs_wait_cfg;
        goto wait_cfg; // FIXME: Gotos are so convenient I love them
    } else if (ctx->cmd_buf_state == cbs_wait_header) {
        // Reading header
        if (len_rx != 0) {
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd + 1, 1);
            --len_rx;
            --len_avail;
            ctx->buffered_length = 2;
            ctx->cmd_buf_state = cbs_wait_cfg;
            goto wait_cfg;
        } else {
            return fcr_incomplete;
        }
    } else if (ctx->cmd_buf_state == cbs_wait_cfg) {
        // Reading configuration section
wait_cfg:
        switch (header->common.op) {
        case op_probe_mem_rw:
            ctx->cmd_buf_wait_payload_size = (header->probe_mem_rw.is_write ? 8 : 4);
            ctx->cmd_buf_state = cbs_wait_payload;
            goto wait_payload;
        case op_io_manip:
            ctx->cmd_buf_state = cbs_execute;
            return fcr_execute;
        case op_ir_rw:
        case op_dr_rw:
            if (len_avail > 4) { // header(u16) + ir_seq_bitlen(u16)
                const uint32_t consume_len = 4 - ctx->buffered_length;

                lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length,
                                           consume_len);
                len_rx -= consume_len;
                ctx->buffered_length += consume_len;
                ctx->cmd_buf_wait_payload_size = BIT2DWORD(cmd->ir_rw.irseq_len_bits) * 4;
                ctx->cmd_buf_state = cbs_wait_payload;
                CHECK_CMD_LENGTH(ctx);
                goto wait_payload;
            }
            break;
        case op_loopback_test:
            if (len_avail > 6) { // header(u16) + number(u32)
                lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length,
                                           6 - ctx->buffered_length);
                ctx->cmd_buf_state = cbs_execute;
                return fcr_execute;
            }
            break;
        case op_fast_target_mem_write_fastdata:
        case op_fast_target_mem_write: {
            // header(u16) + clkdiv(u16) + len(u32) + cpucore(u16)
            const uint32_t expect_len = header->fast_target_mem_write.chained_core_count == 0 ?
                                        8 : 10; // When chained core count is 0, cpucore is gone
            
            if (len_avail > expect_len) {
                const uint32_t consume_len = expect_len - ctx->buffered_length;

                lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length,
                                           consume_len);
                len_rx -= consume_len;
                ctx->buffered_length += consume_len;
                ctx->cmd_buf_wait_payload_size = cmd->fast_mem_read_at_core.data_len_dword_count *
                                                 4;
                ctx->cmd_buf_state = cbs_wait_payload;
                goto wait_payload;
            }
            break;
        }
        case op_fast_target_mem_read_fastdata:
        case op_fast_target_mem_read:
            if (len_avail > 10) { // header(u16) + clkdiv(u16) + len(u32) + cpucore(u16)
                const uint32_t consume_len = 10 - ctx->buffered_length;

                lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length,
                                           consume_len);
                len_rx -= consume_len;
                ctx->buffered_length += consume_len;
                ctx->cmd_buf_state = cbs_execute;
                return fcr_execute;
            }
            break;
        case op_get_firmware_date:
            ctx->cmd_buf_state = cbs_execute;
            return fcr_execute;
        default:
            return fcr_unknown_cmd;
        }

        // This is where we landed after breaks
        // Since available length is not enough for config section anyways, we consume them all
        lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length, len_rx);
        ctx->buffered_length += len_rx;
        return fcr_incomplete;
    } else if (ctx->cmd_buf_state == cbs_wait_payload) {
wait_payload:
        // Payload isn't long enough anyways, consume them all
        if (len_rx < ctx->cmd_buf_wait_payload_size) {
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length, len_rx);
            ctx->buffered_length += len_rx;
            ctx->cmd_buf_wait_payload_size -= len_rx;
            return fcr_incomplete;
        } else {
            // We can assemble a complete command now
            const uint32_t consume_len = ctx->cmd_buf_wait_payload_size;
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length, consume_len);
            ctx->buffered_length += consume_len;
            ctx->cmd_buf_wait_payload_size = 0;
            ctx->cmd_buf_state = cbs_execute;
            return fcr_execute;
        }
    } else {
        return fcr_corrupt_state;
    }
}

lsejtag_cmd_exec_result lsejtag_execute(lsejtag_ctx *ctx) {
    uint32_t usb_tx_len = 0;
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;

    if (ctx->cmd_buf_state != cbs_execute) {
        return cer_not_ready;
    }

    if (ctx->continuation) {
        return lsejtag_execute_continuation(ctx);
    }

    usb_tx_len = usb_tx_len_required(ctx);
    if (!is_continuation_capable_op(cmd->common.header.op) &&
        usb_tx_len > lsejtag_impl_usbtx_free_len()) {
        // Continuation capable ops are exempted. They do actually handle large amount of data and
        // doesn't have to send all of them in one go.
        return cer_await_usb_tx;
    }

    switch (cmd->common.header.op) {
        case op_probe_mem_rw:
            return cmdimpl_probe_mem_rw(ctx);
        case op_io_manip:
            return cmdimpl_io_manip(ctx);
        case op_ir_rw:
            break;
        case op_dr_rw:
            break;
        case op_loopback_test:
            break;
        case op_fast_target_mem_write_fastdata:
            break;
        case op_fast_target_mem_write:
            break;
        case op_fast_target_mem_read_fastdata:
            break;
        case op_fast_target_mem_read:
            break;
        case op_get_firmware_date:
            return cmdimpl_get_firmware_date(ctx);
    }
}

static inline lsejtag_cmd_exec_result lsejtag_execute_continuation(lsejtag_ctx *ctx) {
    if (!is_continuation_capable_op(ctx->cont_op)) {
        return cer_bad_continue;
    }


}