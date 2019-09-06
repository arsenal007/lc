set (CMAKE_GENERATOR "MSYS Makefiles" CACHE INTERNAL "" FORCE)
#message("generator is set to ${CMAKE_GENERATOR}")
cmake_minimum_required(VERSION 3.7)
project(lcusb VERSION 1.00 LANGUAGES C CXX)

message("CMAKE_CURRENT_LIST_FILE: ${CMAKE_CURRENT_LIST_FILE}")
message("CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
message("CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message("CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")
#include(FindWinHID)
#include(MinGWSearchPathExtras)
find_path(WINHIDAPI_INCLUDE_DIR
		NAMES
		hidapi/hidapi.h
		PATHS
		"${WINHIDAPI_ROOT_DIR}"
		${MINGWSEARCH_INCLUDE_DIRS}
		/mingw
		PATH_SUFFIXES
		include/w32api/ddk
		include/ddk
		ddk)
		
find_path(TCLAP_INCLUDE_DIR
		NAMES
		tclap/CmdLine.h
		PATHS
		"${WINHIDAPI_ROOT_DIR}"
		${CMAKE_SOURCE_DIR}
		/mingw
		PATH_SUFFIXES
		tclap/include)		
set(CMAKE_FIND_STATIC on)
find_library(WINHIDAPI_LIBRARY
			NAMES
			libhidapi.a
			libhidapi.dll.a
			HINTS
			"${WINHID_ROOT_DIR}"
			${MINGWSEARCH_LIBRARY_DIRS}
			/mingw
			PATH_SUFFIXES
			lib
			lib/w32api)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)
#set(CMAKE_CXX_COMPILER clang++-5.0)
set(CMAKE_CXX_STANDARD 17)
set(WARNINGS "-Wall -Wextra -Wpedantic -Wshadow")
#set(SANITIZERS "-fsanitize=address,undefined")
set(CMAKE_CXX_FLAGS "${WARNINGS} ${SANITIZERS}")
# list of source files
set(SOURCE_FILES main.cpp)

set(WINHIDAPI_LIBRARY C:/msys64/mingw64/lib/libhidapi.a)
find_library(SETUPAPI_LIBRARY
			NAMES
			setupapi.a
		    HINTS
			"${WINHID_ROOT_DIR}"
			${MINGWSEARCH_LIBRARY_DIRS}
			/mingw
			/c/msys64/mingw64/x86_64-w64-mingw32/lib
			PATH_SUFFIXES
			lib
			lib/w32api)
			
set(SETUPAPI_LIBRARY C:/msys64/mingw64/x86_64-w64-mingw32/lib/libsetupapi.a)

# this is the "object library" target: compiles the sources only once
add_library(objlib OBJECT ${SOURCE_FILES} )

# shared libraries need PIC
set_property(TARGET objlib PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src ${WINHIDAPI_INCLUDE_DIR} ${TCLAP_INCLUDE_DIR} )
add_executable(lc $<TARGET_OBJECTS:objlib> )
#target_include_directories(C PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../src> )
message("WINHIDAPI_LIBRARY: ${WINHIDAPI_LIBRARY}")
message("WINHID_ROOT_DIR: ${WINHID_ROOT_DIR}")
message("MINGWSEARCH_LIBRARY_DIRS: ${MINGWSEARCH_LIBRARY_DIRS}")
message("WINHID_ROOT_DIR: ${WINHID_ROOT_DIR}")
message("CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message("CMAKE_LIBRARY_PATH: ${CMAKE_LIBRARY_PATH}")

target_link_libraries(lc ${WINHIDAPI_LIBRARY} ${SETUPAPI_LIBRARY})
target_link_libraries(lc -static)
#add_library(lcusb STATIC $<TARGET_OBJECTS:objlib>)
