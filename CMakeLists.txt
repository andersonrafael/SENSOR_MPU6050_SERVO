# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(u3-i4-mpu6050 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


# Add all source files to the project
file(GLOB SOURCE_FILES "src/*.c")
file(GLOB_RECURSE DRIVER_FILES "src/drivers/*.c")

# Add executable. Default name is the project name, version 0.1

# add_executable(u3-i4-mpu6050 u3-i4-mpu6050.c )
add_executable(u3-i4-mpu6050 
    ${SOURCE_FILES} 
    ${DRIVER_FILES}
)

pico_set_program_name(u3-i4-mpu6050 "u3-i4-mpu6050")
pico_set_program_version(u3-i4-mpu6050 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(u3-i4-mpu6050 0)
pico_enable_stdio_usb(u3-i4-mpu6050 1)


# Add the standard library to the build
target_link_libraries(u3-i4-mpu6050
        pico_stdlib
        hardware_i2c
        hardware_pwm
    )

# Add the standard include files to the build
target_include_directories(u3-i4-mpu6050 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/src/drivers
)

pico_add_extra_outputs(u3-i4-mpu6050)