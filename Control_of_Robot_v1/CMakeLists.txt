cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)


project(myRealMadridRobocup)



pico_sdk_init()

add_executable(myrealmadridrobocup
    main.cpp
    bluetooth.cpp
    control.cpp
    motor_control.cpp
    wheel_control.cpp
    imu.cpp
    dribbleo.cpp
)

pico_enable_stdio_uart(myrealmadridrobocup 1)
pico_enable_stdio_usb(myrealmadridrobocup 1)



target_link_libraries(myrealmadridrobocup pico_stdlib pico_sync pico_multicore hardware_pwm hardware_uart hardware_sync hardware_resets hardware_i2c)

pico_add_extra_outputs(myrealmadridrobocup)