# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Find the Eigen3 package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(QuatFilter C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add the include directory
include_directories(inc)

# Add executable with source files from 'src' folder
file(GLOB SOURCES "src/*.cpp")
add_executable(QuatFilter main.cpp ${SOURCES})

# pull in common dependencies
target_link_libraries(QuatFilter
    pico_stdlib
    hardware_pwm
    hardware_i2c
)

pico_enable_stdio_usb(QuatFilter 1)
pico_enable_stdio_uart(QuatFilter 1)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(QuatFilter)

target_include_directories(QuatFilter PRIVATE ${EIGEN3_INCLUDE_DIR})

# Disable annoying cmake warning
# Disable specific warning in GCC
target_compile_options(QuatFilter PRIVATE -Wno-psabi)
