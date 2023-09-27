cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)


project(control_4_motores)



pico_sdk_init()

add_executable(control_4_motores
control_4_motores.c
)

pico_enable_stdio_usb(control_4_motores 1)

target_link_libraries(control_4_motores pico_stdlib pico_multicore hardware_adc hardware_pwm)

pico_add_extra_outputs(control_4_motores)