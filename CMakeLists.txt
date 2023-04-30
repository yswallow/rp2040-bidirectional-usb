cmake_minimum_required(VERSION 3.13)
set(target_name split_by_usb)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

#include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
project(pio_usb)

pico_sdk_init()

# add pico-pio-usb library for examples
set(PICO_PIO_USB_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/Pico-PIO-USB")
# the second argument to add_subdirectory is needed here because this isn't
# a subdirectory, it's out of tree.
add_subdirectory(${PICO_PIO_USB_DIR} pico_pio_usb)

add_executable(${target_name}
    split_by_usb.c
)

target_link_libraries(${target_name} 
    tinyusb_device
    tinyusb_host
    pico_stdlib
    pico_multicore
    pico_pio_usb
)

# needed so tinyusb can find tusb_config.h
target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)

pico_add_extra_outputs(${target_name})
