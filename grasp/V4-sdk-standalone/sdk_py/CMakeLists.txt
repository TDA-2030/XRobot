cmake_minimum_required(VERSION 3.4...3.18)
project(rm_sdk)
find_package(Python COMPONENTS Interpreter Development)

add_subdirectory(${CMAKE_SOURCE_DIR}/pybind11)

file(GLOB sdk_common
    "${CMAKE_SOURCE_DIR}/../sdk_common/*.h"
    "${CMAKE_SOURCE_DIR}/../sdk_common/*.cpp"
    "${CMAKE_SOURCE_DIR}/../sdk_common/libmodbus/*.h"
    "${CMAKE_SOURCE_DIR}/../sdk_common/libmodbus/*.c"
)

pybind11_add_module(motormaster wrapper.cpp ${sdk_common})

target_compile_definitions(motormaster PRIVATE VERSION_INFO=${SDK_VERSION_INFO})


MESSAGE(STATUS "operation system is ${CMAKE_SYSTEM}")

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
	MESSAGE(STATUS "current platform: Linux ")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
	MESSAGE(STATUS "current platform: Windows")
    target_link_libraries(motormaster PRIVATE "ws2_32" )
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
	MESSAGE(STATUS "current platform: FreeBSD")
ELSE ()
	MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")


if(MSVC)
	message("msvc cpp11")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest")
else(MSVC)
	message("gnu cpp11")
	target_compile_features(motormaster PRIVATE cxx_std_17)
endif(MSVC)