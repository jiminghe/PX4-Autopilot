cmake_minimum_required(VERSION 3.5)
project(libxsens CXX)

include_directories(.)

set(SOURCES
    xbus.cpp
    xbus_parser.cpp
    xbus.h
    xbus_message_id.h
    xbus_parser.h
)

add_library(libxsens ${SOURCES})
add_dependencies(libxsens prebuild_targets)

target_compile_options(libxsens
    PRIVATE
        -Wno-unused-parameter
        -Wno-unused-variable
        -Wno-sign-compare
        -Wno-cast-align
        -Wno-double-promotion
)

if("${PX4_PLATFORM}" MATCHES "nuttx")
    target_compile_definitions(libxsens PUBLIC __NUTTX__)
endif()
