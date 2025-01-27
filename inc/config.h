
#pragma once

#define DISABLE_DBGPRINT

#ifndef FIRMWARE_VERSION_STR
#define FIRMWARE_VERSION_STR            "0.1.0"
#endif

#ifndef DISABLE_DBGPRINT
#define DbgPrint printf
#else
#define DbgPrint(...)
#endif

#define EJTAG_PIN_TRST                  8
#define EJTAG_PIN_TDI                   9
#define EJTAG_PIN_TDO                   10
#define EJTAG_PIN_TMS                   11
#define EJTAG_PIN_TCK                   12
#define EJTAG_PIN_BRST                  13
#define EJTAG_PIN_DINT                  14
#define EJTAG_PIN_VIO                   26
#define EJTAG_PIN_TDO_SAMPLE_TIME       6

#define EJTAG_SM_START_MASK             0x7
#define EJTAG_SM_START_MASK_NO_TDO      0x3

#define WS2812_PIN                      16



