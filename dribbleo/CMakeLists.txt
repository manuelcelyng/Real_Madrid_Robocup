set(PROJECT_NAME BluetoothC)

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(${PROJECT_NAME})

pico_sdk_init()

add_executable(${PROJECT_NAME}
    bluetooth.cpp
    dribbleo.cpp
    main.cpp
)

pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

target_link_libraries(${PROJECT_NAME} pico_stdlib)


pico_add_extra_outputs(${PROJECT_NAME}) 