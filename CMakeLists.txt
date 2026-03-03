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
        Core/Inc/Servo_Axon_Mini_MKII.h
        Core/Inc/timing.h
        Core/Inc/Math/Math.h
        Core/Inc/TEST_FUNCTIONS.h
        Core/Inc/ism6hg256x_reg.h
        Core/Inc/IMU_ISM6HGx.h
        Core/Inc/Math/Vector3D.h
        Core/Src/Math/Vector3D.cpp
        Core/Src/interrupt_override.cpp
        Core/Inc/Sensors.h
        Core/Inc/Math/Math.h
        Core/Inc/bmi3.h
        Core/Inc/bmi3_defs.h
        Core/Inc/bmi323.h
        Core/Inc/bmi323_defs.h
        Core/Inc/SIMU_BMI323.h
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
        Core/Src/Servo_Axon_Mini_MKII.cpp
        Core/Src/timing.cpp
        Core/Src/Math/Math.cpp
        Core/Src/TEST_FUNCTIONS.cpp
        Core/Src/main.cpp
        Core/Src/ism6hg256x_reg.c
        Core/Src/IMU_ISM6HGx.cpp
        Core/Src/bmi3.c
        Core/Src/bmi323.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
