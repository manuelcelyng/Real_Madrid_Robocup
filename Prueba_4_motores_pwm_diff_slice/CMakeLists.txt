cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)


project(motor_control)



pico_sdk_init()

add_executable(motor_control
motor_control.c
bluetooth.c
)

pico_enable_stdio_uart(motor_control 1)
pico_enable_stdio_usb(motor_control 1)



target_link_libraries(motor_control pico_stdlib pico_multicore hardware_pwm hardware_uart hardware_sync hardware_resets)

pico_add_extra_outputs(motor_control)