cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

find_package(glwin)
find_package(sbdimp)
find_package(sbsprgraphics)

set_property(GLOBAL PROPERTY USE_FOLDERS ON) 
set(CONF_FILES
	../../bin/conf/robotarm/setting.xml
	../../bin/conf/robotarm/welding.xml
	../../bin/conf/robotarm/scene_reaching2d.xml
	../../bin/conf/robotarm/scene_reaching3d.xml
	../../bin/conf/robotarm/scene_welding.xml
)
source_group("conf" FILES ${CONF_FILES})

add_executable(RobotArm
  main.cpp
  mymodule.cpp
  RobotArm.cpp
  Workspace.cpp
  mymodule.h
  RobotArm.h
  Workspace.h
  ${CONF_FILES}
)

set_target_properties(RobotArm PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(RobotArm PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(RobotArm PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(RobotArm PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(RobotArm console)
target_link_libraries(RobotArm Springhead::Springhead)
target_link_libraries(RobotArm glwin)
target_link_libraries(RobotArm dimp)
target_link_libraries(RobotArm Scenebuilder)
target_link_libraries(RobotArm sbdimp)
target_link_libraries(RobotArm sbsprgraphics)
target_link_libraries(RobotArm SDL2::SDL2)
target_link_libraries(RobotArm SDL2_ttf::SDL2_ttf)
target_link_libraries(RobotArm GLEW::glew)
target_link_libraries(RobotArm           ${EXPAT_LIBRARIES})
target_link_libraries(RobotArm           ${LAPACK_LIBRARIES})

if(UNIX)
	target_link_libraries(RobotArm dl)
	target_link_libraries(RobotArm -L${SDL2_LIBDIR} -lSDL2 -lSDL2_ttf)
	target_link_libraries(RobotArm -lpng)
endif()

target_link_libraries(RobotArm debug     ${GLUT_glut_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${GLUT_glut_LIBRARY_RELEASE})
target_link_libraries(RobotArm           ${OPENGL_LIBRARIES})
target_link_libraries(RobotArm           ${Boost_LIBRARIES})
target_link_libraries(RobotArm           ${OpenCV_LIBRARIES})
target_link_libraries(RobotArm           ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(RobotArm           ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(RobotArm           ${MKL_LIB_DIR}/mkl_sequential.lib)
target_link_libraries(RobotArm debug     ${PNG_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${PNG_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${ZLIB_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${ZLIB_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${FREETYPE_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${FREETYPE_LIBRARY_RELEASE})

