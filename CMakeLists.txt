cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(robot_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()



add_library(sts_servo_driver STSServoDriver/STSServoDriver.cpp)
target_link_libraries(sts_servo_driver pico_stdlib)

add_executable(robot_pico main.cpp)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(robot_pico pico_stdlib)
target_link_libraries(robot_pico sts_servo_driver)

# Enables serial output via USB and UART.
# pico_enable_stdio_usb(test 1)
# pico_enable_stdio_uart(test 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(robot_pico)
