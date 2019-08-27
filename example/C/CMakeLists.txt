cmake_minimum_required(VERSION 3.7)
project(lcusb VERSION 1.00 LANGUAGES C CXX)
message("${CMAKE_CURRENT_SOURCE_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
include(FindWinHID)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)
#set(CMAKE_CXX_COMPILER clang++-5.0)
set(CMAKE_CXX_STANDARD 17)
set(WARNINGS "-Wall -Wextra -Wpedantic -Wshadow")
#set(SANITIZERS "-fsanitize=address,undefined")
set(CMAKE_CXX_FLAGS "${WARNINGS} ${SANITIZERS}")
# list of source files
set(SOURCE_FILES ../../src/hid.c main.cpp)

# this is the "object library" target: compiles the sources only once
add_library(objlib OBJECT ${SOURCE_FILES} )

# shared libraries need PIC
set_property(TARGET objlib PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../src)
add_executable(C $<TARGET_OBJECTS:objlib> )
#target_include_directories(C PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../src> )
#target_link_libraries(lcusb ${WINHID_LIBRARIES})
#add_library(lcusb STATIC $<TARGET_OBJECTS:objlib>)
