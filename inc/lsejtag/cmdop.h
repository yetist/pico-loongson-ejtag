
#pragma once

#include "lsejtag/lsejtag.h"

#define DEF_COMMAND(X) lsejtag_cmd_exec_result cmdimpl_##X(lsejtag_ctx *ctx)

DEF_COMMAND(probe_mem_rw);
DEF_COMMAND(io_manip);
DEF_COMMAND(get_firmware_date);
