cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(test_servo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(STSServoDriver)

add_executable(test_servo simple_velocity.cpp)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(test_servo pico_stdlib sts_servo_driver)

# Enables serial output via USB and UART through stdio.
pico_enable_stdio_usb(test_servo 1)
pico_enable_stdio_uart(test_servo 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(test_servo)
