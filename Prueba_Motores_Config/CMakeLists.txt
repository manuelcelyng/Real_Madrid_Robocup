cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)


project(myBlink)



pico_sdk_init()

add_executable(frecuPoli
    frecuPoli.c
)

pico_enable_stdio_usb(frecuPoli 1)

target_link_libraries(frecuPoli pico_stdlib pico_multicore hardware_adc hardware_pwm)

pico_add_extra_outputs(frecuPoli)