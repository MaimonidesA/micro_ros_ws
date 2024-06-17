cmake_minimum_required(VERSION 3.12)

# Name project
SET(ProjectName SPI_Peripherals_pico)

# Set any variables required for importing libraries

# Import those libraries
include(pico_sdk_import.cmake)

# Define project
project(${ProjectName})

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add subdirectories
add_subdirectory(src)

