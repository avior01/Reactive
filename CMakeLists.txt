#
# Copyright 2014-2015 Ettus Research LLC
# Copyright 2018 Ettus Research, a National Instruments Company
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

cmake_minimum_required(VERSION 3.8)
project(EXTENDER CXX) 

### Configure Compiler ########################################################
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD" AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    set(CMAKE_EXE_LINKER_FLAGS "-lthr ${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}")
endif()

### Set up build environment ##################################################
# Choose a static or shared-library build (shared is default, and static will
# probably need some special care!)
# Set this to ON in order to link a static build of UHD:
option(UHD_USE_STATIC_LIBS ON) 

# To add UHD as a dependency to this project, add a line such as this:
find_package(UHD 4.1.0 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)


# The version in  ^^^^^  here is a minimum version.
# To specify an exact version:
#find_package(UHD 4.0.0 EXACT REQUIRED)

#find_library(libSnowFlakesLib libSnowFlakesLib)

# This example also requires Boost.
# Set components here, then include UHDBoost to do the actual finding
set(UHD_BOOST_REQUIRED_COMPONENTS
    program_options
    system
    filesystem
    thread    
)
set(BOOST_MIN_VERSION 1.65)
set(IPP_ROOT "/opt/intel/oneapi/ipp/latest")
set(IPP_INCLUDE_DIR ${IPP_ROOT}/include)
set(IPP_LIB_DIR ${IPP_ROOT}/lib/intel64)

include(UHDBoost)

# Check if IPP directories exist
if(NOT EXISTS ${IPP_INCLUDE_DIR})
    message(FATAL_ERROR "Intel IPP include directory not found at ${IPP_INCLUDE_DIR}")
endif()

if(NOT EXISTS ${IPP_LIB_DIR})
    message(FATAL_ERROR "Intel IPP library directory not found at ${IPP_LIB_DIR}")
endif()

# need these include and link directories for the build
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ${IPP_INCLUDE_DIR})
    
link_directories(${IPP_LIB_DIR})


### Make the executable #######################################################
add_executable(reactive reactive.cpp)


set(Boost_USE_STATIC_LIBS   ON)

#set(CMAKE_BUILD_TYPE "Release")

#if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#    set(UHD_LOG_MIN_LEVEL "fatal" CACHE STRING "Set UHD log level to {trace, debug, info, warning, error, fatal}")
#    set(UHD_LOG_CONSOLE_DISABLE "OFF" CACHE BOOL "Disable UHD logging to stderr")
#    set(UHD_LOG_FILE_LEVEL "fatal" CACHE STRING "SET UHD file logging level to {trace, debug, info, warning, error, fatal}")
#    set(UHD_LOG_CONSOLE_LEVEL "fatal" CACHE STRING "SET UHD file logging level to {trace, debug, info, warning, error, fatal}")    
#else()
#    set(UHD_LOG_MIN_LEVEL "fatal" CACHE STRING "Set UHD log level to {trace, debug, info, warning, error, fatal}")
#    set(UHD_LOG_CONSOLE_DISABLE "OFF" CACHE BOOL "Disable UHD logging to stderr")
#    set(UHD_LOG_FILE_LEVEL "fatal" CACHE STRING "SET UHD file logging level to {trace, debug, info, warning, error, fatal}")
#    set(UHD_LOG_CONSOLE_LEVEL "fatal" CACHE STRING "SET UHD file logging level to {trace, debug, info, warning, error, fatal}")
#endif()
 
# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
if(NOT UHD_USE_STATIC_LIBS)
    message(STATUS "Linking against shared UHD library.")
    target_link_libraries(reactive ${UHD_LIBRARIES} ${Boost_LIBRARIES} ipps ippcore)

    #message(STATUS "Linking against shared UHD library.")
    #target_link_libraries(MainForTest ${UHD_LIBRARIES} ${Boost_LIBRARIES} libSnowFlakesLib_ver6.so cufft libcudart.so)

# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
else(NOT UHD_USE_STATIC_LIBS)
    message(STATUS "Linking against static UHD library.")
    target_link_libraries(rx_to_file
        # We could use ${UHD_LIBRARIES}, but linking requires some extra flags,
        # so we use this convenience variable provided to us
        ${UHD_STATIC_LIB_LINK_FLAG}
        # Also, when linking statically, we need to pull in all the deps for
        # UHD as well, because the dependencies don't get resolved automatically
        ${UHD_STATIC_LIB_DEPS}
    )
   
endif(NOT UHD_USE_STATIC_LIBS)

### Once it's built... ########################################################
# Here, you would have commands to install your program.
# We will skip these in this example.