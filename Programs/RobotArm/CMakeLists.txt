cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

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
  ${CONF_FILES}
)

set_target_properties(RobotArm PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(RobotArm PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(RobotArm PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(RobotArm PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(RobotArm console)
target_link_libraries(RobotArm debug     ${SPRINGHEAD_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${SPRINGHEAD_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${GLWIN_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${GLWIN_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${DIMP_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${DIMP_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${SCENEBUILDER_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${SCENEBUILDER_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${SCENEBUILDER_DIMP_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${SCENEBUILDER_DIMP_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${SCENEBUILDER_SPRGRAPHICS_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${SCENEBUILDER_SPRGRAPHICS_LIBRARY_RELEASE})
target_link_libraries(RobotArm           ${EXPAT_LIBRARIES})
target_link_libraries(RobotArm           ${LAPACK_LIBRARIES})
#target_link_libraries(RobotArm lapacke)

if(UNIX)
	target_link_libraries(RobotArm dl)
	target_link_libraries(RobotArm -L${SDL2_LIBDIR} -lSDL2 -lSDL2_ttf)
	target_link_libraries(RobotArm -lpng)
endif()

target_link_libraries(RobotArm debug     ${GLUT_glut_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${GLUT_glut_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${GLEW_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${GLEW_LIBRARY_RELEASE})
target_link_libraries(RobotArm           ${OPENGL_LIBRARIES})
target_link_libraries(RobotArm           ${Boost_LIBRARIES})
target_link_libraries(RobotArm           ${OpenCV_LIBRARIES})
target_link_libraries(RobotArm debug     ${SDL2_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${SDL2_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${SDL2_ttf_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${SDL2_ttf_LIBRARY_RELEASE})
target_link_libraries(RobotArm           ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(RobotArm           ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(RobotArm           ${MKL_LIB_DIR}/mkl_sequential.lib)
target_link_libraries(RobotArm debug     ${PNG_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${PNG_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${ZLIB_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${ZLIB_LIBRARY_RELEASE})
target_link_libraries(RobotArm debug     ${FREETYPE_LIBRARY_DEBUG})
target_link_libraries(RobotArm optimized ${FREETYPE_LIBRARY_RELEASE})

