cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Kingfisher-SW)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Inc/GPS.h
#        Core/Inc/ubxlib-master/common/device/api/u_device.h
#        Core/Inc/ubxlib-master/gnss/api/u_gnss.h
#        Core/Inc/ubxlib-master/port/api/u_port.h
#        Core/Inc/ubxlib-master/port/api/u_port_i2c.h
#        Core/Inc/ubxlib-master/common/error/api/u_error_common.h
#        Core/Inc/ubxlib-master/common/device/api/u_device_serial.h
#        Core/Inc/ubxlib-master/common/type/api/u_common_spi.h
#        Core/Inc/ubxlib-master/common/device/api/u_device_handle.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Core/Src/Buzzer.cpp
        Core/Src/main.cpp
        Core/Src/GPS.cpp
)

# Link all subfolders in ublox library
file(GLOB_RECURSE UBXLIB_INCLUDE_DIRS
        LIST_DIRECTORIES true
        ${CMAKE_SOURCE_DIR}/Core/Inc/ubxlib-master/*
)

# Filter only directories
set(UBXLIB_DIRS "")
foreach(dir ${UBXLIB_INCLUDE_DIRS})
    if(IS_DIRECTORY ${dir})
        list(APPEND UBXLIB_DIRS ${dir})
    endif()
endforeach()

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/Inc/ubxlib-master
        ${UBXLIB_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    # Add user defined libraries
        stm32cubemx
)
