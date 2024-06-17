cmake_minimum_required(VERSION 3.12)



# Name project
SET(ProjectName pico_def_drive)
include(pico_sdk_import.cmake)

project(${ProjectName} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()


# Set any variables required for importing libraries


# Import those libraries


SET(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/micro_ros_raspberrypi_pico_sdk/" CACHE STRING "Common Lib")
include(micro_ros.cmake)

SET(EIGEN_PATH "${CMAKE_CURRENT_LIST_DIR}/../lib/eigen/" CACHE STRING "Common Lib")
include(eigen_import.cmake)


add_subdirectory(src)

