
#include "lsejtag/config.h"
#include "lsejtag/cmdop.h"
#include "lsejtag/lsejtag_impl.h"

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

    (void) ctx;

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
        case LSEJTAG_PARAM_ADDR_IR_LEN     : ctx->param_ir_len_bits = cmd->data_to_write;
        case LSEJTAG_PARAM_ADDR_IR_SKIP    : ctx->param_ir_skip     = cmd->data_to_write;
        case LSEJTAG_PARAM_ADDR_IR_CONTROL : ctx->param_ir_control  = cmd->data_to_write;
        case LSEJTAG_PARAM_ADDR_IR_DATA    : ctx->param_ir_data     = cmd->data_to_write;
        case LSEJTAG_PARAM_ADDR_IR_ADDRESS : ctx->param_ir_address  = cmd->data_to_write;
        case LSEJTAG_PARAM_ADDR_IR_FASTDATA: ctx->param_ir_fastdata = cmd->data_to_write;
        }
    } else {
        // Reads
        switch (cmd->addr) {
        case 0x40: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_LEN     ; break;
        case 0x44: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_SKIP    ; break;
        case 0x48: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_CONTROL ; break;
        case 0x4c: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_DATA    ; break;
        case 0x50: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_ADDRESS ; break;
        case 0x54: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_FASTDATA; break;
        case LSEJTAG_PARAM_ADDR_IR_LEN     : data_read.u32 = ctx->param_ir_len_bits;
        case LSEJTAG_PARAM_ADDR_IR_SKIP    : data_read.u32 = ctx->param_ir_skip;
        case LSEJTAG_PARAM_ADDR_IR_CONTROL : data_read.u32 = ctx->param_ir_control;
        case LSEJTAG_PARAM_ADDR_IR_DATA    : data_read.u32 = ctx->param_ir_data;
        case LSEJTAG_PARAM_ADDR_IR_ADDRESS : data_read.u32 = ctx->param_ir_address;
        case LSEJTAG_PARAM_ADDR_IR_FASTDATA: data_read.u32 = ctx->param_ir_fastdata;
        default: data_read.u32 = 0;
        }
        lsejtag_impl_usbtx(data_read.bytes, 4);
    }
    cmd_buf_clean(ctx);
    return cer_success;
}
