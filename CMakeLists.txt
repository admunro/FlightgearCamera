cmake_minimum_required(VERSION 3.10)

# Project name and version
project(FlightGearLDP VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Include directories
include_directories(src)

# Source files
set(SOURCES
    src/flightGearLDP.cpp
    src/LDPMaths.cpp
    src/Tracking.cpp
)

# Header files (for IDE support)
set(HEADERS
    src/LDPMaths.hpp
    src/Tracking.hpp
    src/Waypoints.hpp
)

# Create executable
add_executable(flightgearLDP ${SOURCES} ${HEADERS})

# Set output directory
set_target_properties(flightgearLDP PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Create build directory if it doesn't exist
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/build)

# Clean target
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/flightgearLDP
    COMMENT "Cleaning all build artifacts"
)