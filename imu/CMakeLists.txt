cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)


project(imu)

pico_sdk_init()

add_executable(myimu
    imu.c
)

pico_enable_stdio_usb(myimu 1)

target_link_libraries(myimu pico_stdlib)

pico_add_extra_outputs(myimu)