cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_subdirectory(DiMP)

if(BUILD_PROGRAMS)
	add_subdirectory(Console)
endif()

