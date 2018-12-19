cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_library(dimp
	App/App.cpp
	Graph/Avoid.cpp
	Graph/Biped.cpp
	Graph/Connector.cpp
	Graph/Contact.cpp
	Graph/Geometry.cpp
	Graph/Graph.cpp
	Graph/Joint.cpp
	Graph/Match.cpp
	Graph/Node.cpp
	Graph/Object.cpp
	Graph/Task.cpp
	Graph/Timing.cpp
	Graph/Tree.cpp
	Graph/Vehicle.cpp
	Load/Loader.cpp
	Render/Canvas.cpp
	Render/Config.cpp
)

