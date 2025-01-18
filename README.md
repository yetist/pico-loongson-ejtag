

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

It should build, if it doesn't then good luck debugging.

## clangd fix

Due to clangd/LLVM stupidity you have to add `--query-driver=.../your/gnu/toolchain/bin/arm-none-eabi-*.exe` to clangd arguments to make clangd find the system includes, otherwise it recognizes your GNU toolchain as an LLVM toolchain and fails to find headers. See clangd/clangd#2289.
