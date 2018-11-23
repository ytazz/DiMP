cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

project(DiMP)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

set(SPRINGHEAD_DIR "" CACHE PATH "root directory of Springhead")
set(SCENEBUILDER_DIR "" CACHE PATH "root directory of Scenebuilder")

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-reorder")

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${SPRINGHEAD_DIR}/core/include)
include_directories(${SPRINGHEAD_DIR}/core/src)
include_directories(${SCENEBUILDER_DIR}/include)

add_library(dimp
	src/DiMP/App/App.cpp
	src/DiMP/Graph/Avoid.cpp
	src/DiMP/Graph/Biped.cpp
	src/DiMP/Graph/Connector.cpp
	src/DiMP/Graph/Contact.cpp
	src/DiMP/Graph/Geometry.cpp
	src/DiMP/Graph/Graph.cpp
	src/DiMP/Graph/Joint.cpp
	src/DiMP/Graph/Match.cpp
	src/DiMP/Graph/Node.cpp
	src/DiMP/Graph/Object.cpp
	src/DiMP/Graph/Task.cpp
	src/DiMP/Graph/Timing.cpp
	src/DiMP/Graph/Tree.cpp
	src/DiMP/Graph/Vehicle.cpp
	src/DiMP/Load/Loader.cpp
	src/DiMP/Render/Canvas.cpp
	src/DiMP/Render/Config.cpp
)
