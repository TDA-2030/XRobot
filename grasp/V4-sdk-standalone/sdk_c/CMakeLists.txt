cmake_minimum_required(VERSION 3.4...3.18)

file(GLOB sdk_common
    "${CMAKE_SOURCE_DIR}/../sdk_common/*.h"
    "${CMAKE_SOURCE_DIR}/../sdk_common/*.cpp"
    "${CMAKE_SOURCE_DIR}/../sdk_common/libmodbus/*.h"
    "${CMAKE_SOURCE_DIR}/../sdk_common/libmodbus/*.c"
)

add_definitions(-DRM_API_EXPORTS)

add_library(motormaster SHARED ${sdk_common} wrapper.cpp motormaster.h)
add_executable(demo demo.cpp ${sdk_common})