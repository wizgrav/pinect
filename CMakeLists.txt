cmake_minimum_required(VERSION 2.6)

PROJECT(Pinect)

set (PROJECT_VER_MAJOR 1)
set (PROJECT_VER_MINOR 0)
set (PROJECT_VER_PATCH 0)
set (PROJECT_VER 
  "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
include (FindOS)
include (SetupDirectories)
find_package(libusb-1.0 REQUIRED)


######################################################################################
# CMake 
######################################################################################

SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
SET(DOC_OUTPUT_PATH ${CMAKE_BINARY_DIR}/doc)

if(CMAKE_C_FLAGS STREQUAL "")
  set(CMAKE_C_FLAGS "-O2 -g")
endif()
add_definitions(-Wall)


include(ExternalProject)

ExternalProject_Add (
       FREENECT_PROJ
       DOWNLOAD_COMMAND ""
       SOURCE_DIR ${CMAKE_SOURCE_DIR}/deps/libfreenect
       CMAKE_ARGS -DBUILD_EXAMPLES=OFF -DBUILD_C_SYNC=OFF -DBUILD_FAKENECT=OFF
       UPDATE_COMMAND ""
       INSTALL_COMMAND "" )

ExternalProject_Add (
       UVC_PROJ
       DOWNLOAD_COMMAND ""
       SOURCE_DIR ${CMAKE_SOURCE_DIR}/deps/libuvc
       CMAKE_ARGS -DCMAKE_BUILD_TARGET=Static
       UPDATE_COMMAND ""
       INSTALL_COMMAND "" )
       
add_subdirectory (src)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/UninstallTarget.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/UninstallTarget.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/UninstallTarget.cmake)
