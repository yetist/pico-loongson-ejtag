
#include <stdio.h>
#include <string.h>
#include "lsejtag/config.h"
#include "lsejtag/cmdimpl.h"
#include "lsejtag/lsejtag_impl.h"

static uint32_t return_data_len_required(lsejtag_ctx *ctx) {
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;
    
    switch (cmd->common.header.op) {
    case op_probe_mem_rw:
        return cmd->probe_mem_rw.header.is_write ? 0 : 4;
    case op_io_manip:
        return 0;
    // For IR/DR RW this is TDO buffer size required
    case op_ir_rw:
        return cmd->ir_rw.header.read_back ? 0 : (BIT2DWORD(cmd->ir_rw.irseq_len_bits) * 4);
    case op_dr_rw:
        return cmd->dr_rw.header.read_back ? 0 : (BIT2DWORD(cmd->ir_rw.irseq_len_bits) * 4);
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

static inline void wait_until_jtag_periph_free(lsejtag_ctx *ctx) {
    while (ctx->jtagbuf_a.busy || ctx->jtagbuf_b.busy || ctx->tdo_busy);
}

static inline void cmd_buf_clean(lsejtag_ctx *ctx) {
    ctx->buffered_length = 0;
    ctx->cmd_buf_state = cbs_clean;
    ctx->cmd_buf_wait_payload_size = 0;
}

lsejtag_cmd_exec_result cmdimpl_get_firmware_date(lsejtag_ctx *ctx) {
    static const union {
        uint32_t bcd;
        uint8_t bytes[4];
    } version_date = { .bcd = LSEJTAG_FIRMWARE_DATE };

    if (return_data_len_required(ctx) > lsejtag_impl_usbtx_free_len()) {
        return cer_await_usb_tx;
    }

    lsejtag_impl_usbtx(version_date.bytes, 4);
    cmd_buf_clean(ctx);
    return cer_success;
}

lsejtag_cmd_exec_result cmdimpl_probe_mem_rw(lsejtag_ctx *ctx) {
    lsejtag_cmd_probe_mem_rw *cmd = (lsejtag_cmd_probe_mem_rw *)ctx->buffered_cmd;
    union {
        uint32_t u32;
        uint8_t bytes[4];
    } data_read;

    if (cmd->header.is_write) {
        // Writes
        switch (cmd->addr) {
        case 0x81000070: // JTAG timing configuration
            switch ((cmd->data_to_write & 0xffff0000) >> 16) {
            case 1: {
                // Clock divider. Find out highest bit set
                bool found = false;
                for (int i = 0x8000; i != 0; i >>= 1) {
                    if (i & cmd->data_to_write) {
                        ctx->param_jtag_clk_div = i & cmd->data_to_write;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    // In reality this should prevent JTAG port from doing any IO
                    ctx->param_jtag_clk_div = 0;
                }
                break;
            }
            case 2:
                // TDO sampling timing
                ctx->param_jtag_tdo_sample_timing = cmd->data_to_write & 0x3;
                break;
            }
            break;
        case LSEJTAG_PARAM_ADDR_IR_LEN     : ctx->param_ir_len_bits = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_SKIP    : ctx->param_ir_skip     = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_CONTROL : ctx->param_ir_control  = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_DATA    : ctx->param_ir_data     = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_ADDRESS : ctx->param_ir_address  = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_FASTDATA: ctx->param_ir_fastdata = cmd->data_to_write; break;
        }
    } else {
        // Reads
        if (return_data_len_required(ctx) > lsejtag_impl_usbtx_free_len()) {
            return cer_await_usb_tx;
        }

        switch (cmd->addr) {
        case 0x40: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_LEN     ; break;
        case 0x44: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_SKIP    ; break;
        case 0x48: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_CONTROL ; break;
        case 0x4c: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_DATA    ; break;
        case 0x50: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_ADDRESS ; break;
        case 0x54: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_FASTDATA; break;
        case LSEJTAG_PARAM_ADDR_IR_LEN     : data_read.u32 = ctx->param_ir_len_bits; break;
        case LSEJTAG_PARAM_ADDR_IR_SKIP    : data_read.u32 = ctx->param_ir_skip;     break;
        case LSEJTAG_PARAM_ADDR_IR_CONTROL : data_read.u32 = ctx->param_ir_control;  break;
        case LSEJTAG_PARAM_ADDR_IR_DATA    : data_read.u32 = ctx->param_ir_data;     break;
        case LSEJTAG_PARAM_ADDR_IR_ADDRESS : data_read.u32 = ctx->param_ir_address;  break;
        case LSEJTAG_PARAM_ADDR_IR_FASTDATA: data_read.u32 = ctx->param_ir_fastdata; break;
        default: data_read.u32 = 0; break;
        }
        lsejtag_impl_usbtx(data_read.bytes, 4);
    }
    cmd_buf_clean(ctx);
    return cer_success;
}

lsejtag_cmd_exec_result cmdimpl_io_manip(lsejtag_ctx *ctx) {
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;

    if (cmd->io_manip.header.port_id > io_maximum) {
        return cer_success;
    }

    switch ((lsejtag_io)cmd->io_manip.header.port_id) {
    case io_led:
        lsejtag_impl_io_manip(impl_io_led, cmd->io_manip.header.level == 1);
        break;
    case io_trst:
        lsejtag_impl_io_manip(impl_io_trst, cmd->io_manip.header.level == 1);
        break;
    case io_brst:
        lsejtag_impl_io_manip(impl_io_brst, cmd->io_manip.header.level == 1);
        break;
    case io_dint:
        lsejtag_impl_io_manip(impl_io_dint, cmd->io_manip.header.level == 1);
        break;

    case io_fpga_reset:
        // This one probably clears off internal FIFOs or such. Unclear
        break;

    case io_oe:
        // When #OE is pulled low, all input/output are cut off. UNIMPLEMENTED
        break;

    case io_tap_reset: {
        // An absolutely overkill TMS reset sequence
        static const uint32_t tms_seq[] = { 0xffffffff, 0x7fffffff };
        static const uint32_t tdi_seq[] = { 0x0, 0x0 };

        // Since any JTAG sequence will finish pretty fast, we simply wait right here until JTAG
        // peripheral is free, then we send the TAP reset sequence
        wait_until_jtag_periph_free(ctx);

        // Let it go
        lsejtag_impl_run_jtag(tdi_seq, tms_seq, NULL, 64, 0, 0);

        // Also wait until the reset sequence finishes
        wait_until_jtag_periph_free(ctx);
        break;
    }
    default:
    case io_maximum:
        break;
    }

    cmd_buf_clean(ctx);

    return cer_success;
}

lsejtag_cmd_exec_result cmdimpl_irdr_write(lsejtag_ctx *ctx, bool isir) {
    struct lsejtag_jtagbuf_blk *bufblk;
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;
    uint32_t *tmsbuf, *tdibuf, *dataseq;
    uint32_t bitlen, dwordlen, bitshift = 0;
    uint32_t data_dwordlen = BIT2DWORD(cmd->ir_rw.irseq_len_bits);

    // Choose a free JTAG buffer
    if (!ctx->jtagbuf_a.busy) {
        bufblk = &ctx->jtagbuf_a;
    } else if (!ctx->jtagbuf_b.busy) {
        bufblk = &ctx->jtagbuf_b;
    } else {
        return cer_await_jtag_buf;
    }

    tmsbuf = (uint32_t *)&bufblk->tms_data;
    tdibuf = (uint32_t *)&bufblk->tdi_data;
    dataseq = (uint32_t *)&cmd->ir_rw.irseq;

    // See if TDO buffer is enough, if not, force a TDO flush
    if (sizeof(ctx->tdo_in_data) - 4 * ctx->tdo_in_dwords < return_data_len_required(ctx)) {
        ctx->tdo_should_flush = true;
        return cer_await_usb_tx;
    }

    // Calculate bit length, and clear the scratch area
    // TMS: [IR:01100  DR:0100] + [(SeqBitLen-1)x 0] + [110]
    // TDI: [IR:00000  DR:0000] + [DataSeq]           + [00]
    bitlen = 4 + (isir ? 1 : 0) + cmd->ir_rw.irseq_len_bits + 2;
    dwordlen = BIT2DWORD(bitlen);
    memset(tmsbuf, 0, dwordlen * 4);
    memset(tdibuf, 0, dwordlen * 4);
    // We fill this metadata right here for convenience
    bufblk->tdi_bits = bitlen;

    // Fill data
    // First stage
    *tmsbuf = isir ? 0x6 : 0x2;

    // Data sequence
    // The whole data sequence is operated on per DWORD basis
    if (isir) {
        for (uint32_t i = 0; i < data_dwordlen; ++i) {
            *tdibuf |= (dataseq[i]) << 5;
            ++tdibuf;
            *tdibuf = (dataseq[i]) >> 27;
        }
    } else {
        for (uint32_t i = 0; i < data_dwordlen; ++i) {
            *tdibuf |= (dataseq[i]) << 4;
            ++tdibuf;
            *tdibuf = (dataseq[i]) >> 28;
        }
    }

    // Ending TMS sequence
    tmsbuf += cmd->ir_rw.irseq_len_bits / 32;
    bitshift = isir ? 5 : 4;
    bitshift = (bitshift + cmd->ir_rw.irseq_len_bits - 1);
    bitshift %= 32;

    *tmsbuf |= (0x3 << bitshift);
    if (bitshift >= 30) {
        ++tmsbuf;
        *tmsbuf = (0x3 >> (32 - bitshift));
    }

    // Fill in metadata
    if (cmd->ir_rw.header.read_back) {
        bufblk->tdo_skip_bits = isir ? 5 : 4;
        bufblk->tdo_bits = cmd->ir_rw.irseq_len_bits; 
    } else {
        bufblk->tdo_bits = 0;
    }
    
    bufblk->immediate_flush_tdo = (cmd->ir_rw.header.immediate_send_back ? true : false);

    bufblk->prepared = true;

    cmd_buf_clean(ctx);
    return cer_success;
}
