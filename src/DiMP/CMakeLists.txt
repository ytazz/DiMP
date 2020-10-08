cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

set(DIMP_HEADERS
	DiMP.h
	Types.h
	)
set(DIMP_HEADERS_APP
	App/App.h
	)
set(DIMP_HEADERS_GRAPH
	Graph/Avoid.h
	Graph/Biped.h
	Graph/Centroid.h
	Graph/Connector.h
	Graph/Contact.h
	Graph/Geometry.h
	Graph/Graph.h
	Graph/ID.h
	Graph/Joint.h
	Graph/Match.h
	Graph/Node.h
	Graph/Object.h
	Graph/Octtree.h
	Graph/Task.h
	Graph/Timing.h
	Graph/Tree.h
	Graph/Vehicle.h
	)
set(DIMP_HEADERS_LOAD
	Load/Loader.h
	)
set(DIMP_HEADERS_RENDER
	Render/Canvas.h
	Render/Config.h
)

set(DIMP_SOURCES_APP
	App/App.cpp
	)
set(DIMP_SOURCES_GRAPH
	Graph/Avoid.cpp
	Graph/Biped.cpp
	Graph/Centroid.cpp
	Graph/Connector.cpp
	Graph/Contact.cpp
	Graph/Geometry.cpp
	Graph/Graph.cpp
	Graph/Joint.cpp
	Graph/Match.cpp
	Graph/Node.cpp
	Graph/Object.cpp
	Graph/Octtree.cpp
	Graph/Task.cpp
	Graph/Timing.cpp
	Graph/Tree.cpp
	Graph/Vehicle.cpp
	)
set(DIMP_SOURCES_LOAD
	Load/Loader.cpp
	)
set(DIMP_SOURCES_RENDER
	Render/Canvas.cpp
	Render/Config.cpp
)

add_library(dimp
	${DIMP_SOURCES_APP}
	${DIMP_SOURCES_GRAPH}
	${DIMP_SOURCES_LOAD}
	${DIMP_SOURCES_RENDER}
	${DIMP_HEADERS}
	${DIMP_HEADERS_APP}
	${DIMP_HEADERS_GRAPH}
	${DIMP_HEADERS_LOAD}
	${DIMP_HEADERS_RENDER}
	)

target_link_libraries(dimp Springhead::Springhead)
target_link_libraries(dimp Scenebuilder)

install(TARGETS dimp
  EXPORT   dimpExport
  LIBRARY       DESTINATION  lib
  INCLUDES      DESTINATION  include
)
install(FILES ${DIMP_HEADERS}        DESTINATION include/DiMP)
install(FILES ${DIMP_HEADERS_APP}    DESTINATION include/DiMP/App)
install(FILES ${DIMP_HEADERS_GRAPH}  DESTINATION include/DiMP/Graph)
install(FILES ${DIMP_HEADERS_LOAD}   DESTINATION include/DiMP/Load)
install(FILES ${DIMP_HEADERS_RENDER} DESTINATION include/DiMP/Render)

install(EXPORT dimpExport
  FILE         dimpConfig.cmake
  DESTINATION  cmake
)