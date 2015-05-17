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
find_package(libfreenect)

if(LIBFREENECT_FOUND)
message( STATUS "Libfreenect found")
else(LIBFREENECT_FOUND)
message( WARNING "Libfreenect not found")
endif(LIBFREENECT_FOUND)

######################################################################################
# CMake 
######################################################################################

SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
SET(DOC_OUTPUT_PATH ${CMAKE_BINARY_DIR}/doc)

if(CMAKE_C_FLAGS STREQUAL "")
  set(CMAKE_C_FLAGS "-O2 -g")
endif()
add_definitions(-Wall)

add_subdirectory (src)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/UninstallTarget.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/UninstallTarget.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/UninstallTarget.cmake)
