cmake_minimum_required(VERSION 3.12)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/usr/share/pico-sdk")

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# use the directoy name as project name
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId}  C CXX) 

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


#set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()


# Add source directory
add_subdirectory(src)


# Enable usb output, disable uart output
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME}  1) # when not debugging
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 1) # when debugging
