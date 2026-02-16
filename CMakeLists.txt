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
        Core/Inc/u_cfg_override.h
        Core/Inc/ubxlib-master/port/platform/stm32cube/src/u_port.c
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
        Core/Inc/ubxlib-master/gnss/src/u_gnss_pos.c
)

# Find all directories named "api" inside ubxlib
file(GLOB_RECURSE UBXLIB_API_DIRS
        LIST_DIRECTORIES true
        ${CMAKE_SOURCE_DIR}/Core/Inc/ubxlib-master/*/api)

# Keep only directories (GLOB can return files)
set(UBXLIB_API_DIRS_FILTERED "")
foreach(dir ${UBXLIB_API_DIRS})
    if(IS_DIRECTORY ${dir})
        list(APPEND UBXLIB_API_DIRS_FILTERED ${dir})
    endif()
endforeach()

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Core/Inc/ubxlib-master
        ${UBXLIB_API_DIRS_FILTERED}
        Core/Inc/ubxlib-master/port/platform/stm32cube
        Core/Inc/ubxlib-master/cfg
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
