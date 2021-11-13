cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

find_package(glwin)

add_library(console
	module/manager.cpp
	module/module.cpp
	module/rendering.cpp
	module/request.cpp
	module/simulation.cpp
    base/typedefs.h
    module/manager.h
    module/module.h
    module/rendering.h
    module/request.h
    module/simulation.h
)
target_link_libraries(console glwin)
target_link_libraries(console Scenebuilder)
target_link_libraries(console SDL2::SDL2)
target_link_libraries(console SDL2_ttf::SDL2_ttf)

