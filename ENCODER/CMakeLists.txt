cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)


project(encoder)



pico_sdk_init()

add_executable(encoder
    encoder.c
)

pico_enable_stdio_usb(encoder 1)

target_link_libraries(encoder pico_stdlib hardware_i2c)

pico_add_extra_outputs(encoder)