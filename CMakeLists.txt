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

file(GLOB_RECURSE UBXLIB_COMMON_API
        Core/Inc/ubxlib-master/common/*/api/*.h
        Core/Inc/ubxlib-master/common/*.h)

file(GLOB_RECURSE UBXLIB_GNSS_API
        Core/Inc/ubxlib-master/gnss/api/*.h)

file(GLOB_RECURSE UBXLIB_PORT_API
        Core/Inc/ubxlib-master/port/api/*.h
        Core/Inc/ubxlib-master/port/platform/stm32cube/api/*.h)
# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Inc/GPS.h
        Core/Inc/u_cfg_override.h
        Core/Inc/ubxlib-master/ubxlib.h

        ${UBXLIB_COMMON_API}
        ${UBXLIB_GNSS_API}
        ${UBXLIB_PORT_API}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE UBXLIB_COMMON_SRC
        Core/Inc/ubxlib-master/common/*/src/*.c)

file(GLOB_RECURSE UBXLIB_GNSS_SRC
        Core/Inc/ubxlib-master/gnss/src/*.c)

file(GLOB_RECURSE UBXLIB_PORT_SRC
        Core/Inc/ubxlib-master/port/*.c
        Core/Inc/ubxlib-master/port/platform/stm32cube/src/*.c)
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        Core/Src/Buzzer.cpp
        Core/Src/main.cpp
        Core/Src/GPS.cpp
        ${UBXLIB_COMMON_SRC}
        ${UBXLIB_GNSS_SRC}
        ${UBXLIB_PORT_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/Inc/ubxlib-master
        Core/Inc/ubxlib-master/port/platform/stm32cube
        Core/Inc/ubxlib-master/cfg
        Core/Inc/ubxlib-master/port/api
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
        U_CFG_OVERRIDE
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    # Add user defined libraries
        stm32cubemx
)
