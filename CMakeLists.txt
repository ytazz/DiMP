cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

project(DiMP)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX d)
endif()

set(SPRINGHEAD_DIR           "" CACHE PATH "root directory of Springhead")
set(SCENEBUILDER_INCLUDE_DIR "" CACHE PATH "Scenebuilder include directory")
set(SCENEBUILDER_LIB_DIR     "" CACHE PATH "Scenebuilder library directory")
set(GLWIN_DIR                "" CACHE PATH "root directory of glwin")
set(GLWIN_LIB_DIR            "" CACHE PATH "directory where glwin libraries are located")
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
find_package(Boost REQUIRED COMPONENTS regex thread chrono)
find_package(SDL2)
find_package(PNG)

add_definitions(-DFREEGLUT_LIB_PRAGMAS=0)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/Console)
include_directories(${SPRINGHEAD_DIR}/core/include)
include_directories(${SPRINGHEAD_DIR}/core/src)
include_directories(${SCENEBUILDER_INCLUDE_DIR})
include_directories(${SCENEBUILDER_INCLUDE_DIR}/adaptors)
include_directories(${GLWIN_DIR})
include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${MKL_INCLUDE_DIR})

link_directories(${SPRINGHEAD_DIR}/core/lib)
link_directories(${SCENEBUILDER_LIB_DIR})
link_directories(${SCENEBUILDER_LIB_DIR}/adaptors/DiMP)
link_directories(${SCENEBUILDER_LIB_DIR}/adaptors/SprGraphics)
link_directories(${GLWIN_LIB_DIR})

## print all variables
get_cmake_property(_variableNames VARIABLES)
foreach(_variableName ${_variableNames})
	message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

add_subdirectory(src)

if(BUILD_EXAMPLES)
	add_subdirectory(Examples)
endif()

if(BUILD_PROGRAMS)
	add_subdirectory(Programs)
endif()


