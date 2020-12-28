cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

project(DiMP)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX d)
endif()

set(MKL_INCLUDE_DIR          "" CACHE PATH "MKL include directory")

set(BUILD_EXAMPLES false CACHE BOOL "build example projects or not")
set(BUILD_PROGRAMS false CACHE BOOL "build program projects or not")

if(UNIX)
	set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-reorder")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
	set(CMAKE_CXX_FLAGS_DEBUG "-g")
	set(CMAKE_CXX_FLAGS_RELEASE "-O2")
endif()

find_package(OpenGL)
find_package(GLUT)
find_package(GLEW)
find_package(EXPAT)
find_package(LAPACK)
find_package(Freetype)
find_package(Springhead)
find_package(Scenebuilder)
find_package(SDL2     REQUIRED COMPONENTS SDL2::SDL2)
find_package(SDL2_ttf REQUIRED COMPONENTS SDL2_ttf::SDL2_ttf)

set(Boost_USE_STATIC_LIBS        ON)
set(Boost_USE_MULTITHREADED      ON)
find_package(Boost REQUIRED COMPONENTS regex thread chrono serialization)

find_package(PNG)

add_definitions(-DFREEGLUT_LIB_PRAGMAS=0)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/Console)
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${MKL_INCLUDE_DIR})

## print all variables
get_cmake_property(_variableNames VARIABLES)
foreach(_variableName ${_variableNames})
	message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

if(BUILD_PROGRAMS OR BUILD_EXAMPLES)
	set(MKL_LIB_DIR                              "" CACHE PATH "path to MKL lib directory")
endif()

if(BUILD_PROGRAMS)

endif()

set(BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)
if(UNIX)
	set(BIN_INSTALL_DIR ${BIN_DIR})
else()
	if(${CMAKE_GENERATOR_PLATFORM} STREQUAL "x64")
		set(BIN_INSTALL_DIR ${BIN_DIR}/x64)
	else()
		set(BIN_INSTALL_DIR ${BIN_DIR}/x86)
	endif()
endif()

add_subdirectory(src)

if(BUILD_EXAMPLES)
	add_subdirectory(Examples)
endif()

if(BUILD_PROGRAMS)
	add_subdirectory(Programs)
endif()


