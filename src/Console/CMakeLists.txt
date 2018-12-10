cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_library(console
	module/manager.cpp
	module/module.cpp
	module/rendering.cpp
	module/request.cpp
	module/simulation.cpp
)

