# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

set(FREERTOS_KERNEL_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/FreeRTOS-Kernel")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

include(FreeRTOS_Kernel_import.cmake)

project(FreeRTOS_ex1 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
if (TARGET tinyusb_device)
        add_executable(FreeRTOS_ex1 
        main.cpp
        bluetooth.cpp
        control.cpp
        motor_control.cpp
        wheel_control.cpp
        imu.cpp 
        dribbleo.cpp
        )
        target_include_directories(FreeRTOS_ex1 PUBLIC ./)

        pico_set_program_name(FreeRTOS_ex1 "FreeRTOS_ex1")
        pico_set_program_version(FreeRTOS_ex1 "0.1")

        # Add any user requested libraries
        target_link_libraries(FreeRTOS_ex1
                FreeRTOS-Kernel
                FreeRTOS-Kernel-Heap4
                pico_stdlib
                pico_multicore
                pico_sync
                hardware_pwm
                hardware_uart
                hardware_sync
                hardware_resets
                hardware_i2c
        )

        pico_enable_stdio_uart(FreeRTOS_ex1 0)
        pico_enable_stdio_usb(FreeRTOS_ex1 1)
        
        pico_add_extra_outputs(FreeRTOS_ex1)

elseif(PICO_ON_DEVICE)
        message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
