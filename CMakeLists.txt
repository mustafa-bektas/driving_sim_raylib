cmake_minimum_required(VERSION 3.10)
project(DrivingSimulator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find raylib package
find_package(raylib QUIET)
if (NOT raylib_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 4.5.0
    )
    FetchContent_MakeAvailable(raylib)
endif()

# Add executable
add_executable(driving_simulator driving_simulator.cpp)

# Link libraries
target_link_libraries(driving_simulator raylib)

# Additional linking options for specific platforms
if (APPLE)
    target_link_libraries(driving_simulator "-framework IOKit")
    target_link_libraries(driving_simulator "-framework Cocoa")
    target_link_libraries(driving_simulator "-framework OpenGL")
endif()

if (MSVC)
    # Visual Studio compiler
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()