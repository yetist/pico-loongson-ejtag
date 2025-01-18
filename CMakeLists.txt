
cmake_minimum_required(VERSION 3.21)
set(PICO_TOOLCHAIN_PATH $ENV{PICO_SDK_COMPILER_PATH})
include($ENV{PICO_SDK_BASE}/external/pico_sdk_import.cmake)

project(pico-loongson-ejtag LANGUAGES C CXX ASM)
pico_sdk_init()

file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.c)

include_directories(${CMAKE_CURRENT_LIST_DIR}/inc)
add_executable(${PROJECT_NAME}
    ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    hardware_pio
    hardware_dma
    pico_stdlib
    pico_unique_id
    tinyusb_device
    tinyusb_board
)

pico_generate_pio_header(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/pio/ejtag.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio
)

pico_add_extra_outputs(${PROJECT_NAME})
