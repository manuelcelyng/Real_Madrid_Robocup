cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)


project(control)

pico_sdk_init()

add_executable(mycontrol
    control.c
)

pico_enable_stdio_usb(mycontrol 1)

target_link_libraries(mycontrol pico_stdlib)

pico_add_extra_outputs(mycontrol)