
# Pico Loongson EJTAG

Welcome aboard! This is an exciting project for fellow Loongson embedded developer hobbyists. It is a reimplementation of Loongson's EJTAG probe box, and aims to be functionally equivalent and host-software compatible with the official implementation.

My initial intent of building this project is help expanding the ecosystem of Loongson hardware. We know that Loongson made impressive improvements on dedsktop-class processors like 3A6000, but these systems are relatively bulky, expensive and still not *quite* comparable with modern day Western-world-produced hardwares and made a lot people refrain from buying them (myself included). But this has changed, thanks to the emergence of 2K0300 boards. This is an SoC that has comparable CoreMark to popular chips like iMX6ULL, can run both Linux and baremetal, sells dirt cheap (simple SBC available for 100 RMB) and is really small in size, effectively acting as a portal to enter LoongArch world with little cost. Unfortunately, I'm more of a "baremetal embedded developer" and Linux does not serve everyone's need either. While running baremetal applications is largely possible on 2K0300, the debug probe for flashing its firmware always costs a ton. Since Loongson mainly focused on their desktop chips previously, they aren't putting much attention to the embedded world (2K0300 only had an incomplete SDK, made by third-party developer, whose IDE shamely runs only on Windows machines) and I think it's time to prove that we embedded developers can pave our own way for the open LoongArch embedded ecosystem.

This version of the firmware is largely based on the reverse engineering of a probe containing 20210129 version FPGA firmware and the `la_dbg_tool_usb.exe` tool, an independent implementation of the host-probe communication protocol.

***The current version of firmware has a limited feature set (fast memory R/W is not implemented yet, and flashing depends on it.) We will eventually reach there!***

# Completeness

Not all commands are currently implemented and this may render some functions of debugger unusable:

- 0xe, 0xf: Fast memory read
  
  Bulk memory dump command will be unusable

Some commands are not thoroughly tested and may malfunction on some machines:

- 0xc, 0xd: Fast memory write

  Not tested on any other machines than 2K0300, multi-core system compatibility is unknown. Only SPI Flash programming is tested on 2K0300.

# Implementation

This firmware uses a Waveshare RP2040-Zero board, anyone can build it with very little cost. The following pins on the board are used:

|RP2040 GPIO Number|JTAG Signal|Comments|
|-|-|-|
|8|#TRST||
|9|TDI||
|10|TDO||
|11|TMS||
|12|TCK||
|13|#BRST||
|14|#DINT||
|26|VIO|Analog pin|

The onboard WS2812 (on GPIO16) is used as indicator. UART0 on GPIO0/1 are used and can output some debug messages during development.

The `src/lsejtag` and `inc/lsejtag` folder contains most platform-independent logic that processes host software commands, that I call `LS-EJTAG` but has not released as a separate package. `src/ejtag_impl.c` contains the platform-specific code that must be implemented when porting to other chips.

I have an HPM5301 design in the brewing, mainly aimed to exploit the High-Speed USB, full with OSL-W-2.0 licensed hardware design and GPL-3.0 licensed firmware. Stay tuned.

# Building

Because I didn't want to install the huge SDK package, I built my own SDK workflow on Windows, so you should follow my scheme to get it working.

## picotool installation

`picotool` should be installed to `C:\Program Files\CMake\picotool\`, in which you can find picotool's CMake target files, etc.

`pioasm` as well.

Adding the directory path to `CMAKE_PREFIX_PATH` env also works, you may choose one method depends on your preference.

## Environment

- `PICO_SDK_BASE` should be set to a directory which is the direct clone of pico-sdk repository.
- `PICO_SDK_COMPILER_PATH` should be set to a directory which contains the MCU compiler to use (contains `arm-none-eabi-gcc.exe`, etc)

## Open in VSCode to build

You can set the environment in Workspace settings. Now choose "ARM GCC (Pico)" toolchain when prompted.

It should build, if it doesn't then good luck debugging...

## clangd fix

Due to clangd/LLVM stupidity you have to add `--query-driver=.../your/gnu/toolchain/bin/arm-none-eabi-*.exe` to clangd arguments to make clangd find the system includes, otherwise it recognizes your GNU toolchain as an LLVM toolchain and fails to find headers. See clangd/clangd#2289.

# Licensing

All the code are distributed under the GPL-3.0-only license, except for those files that contain explicit copyright claims from others.
