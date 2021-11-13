cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

find_package(glwin)
find_package(sbdimp)
find_package(sbsprgraphics)
find_package(PNG)
find_package(Curses)
#find_package(Freetype)

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

target_link_libraries(RobotArm
    PRIVATE
    console
    Springhead::Springhead
    glwin
    dimp
    Scenebuilder
    sbdimp
    sbsprgraphics
    SDL2::SDL2
    SDL2_ttf::SDL2_ttf
    GLUT::GLUT
    GLEW::glew
    ${EXPAT_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${PNG_LIBRARIES}
    ${ZLIB_LIBRARIES}
    ${CURSES_LIBRARIES}
    )

if(UNIX)
	target_link_libraries(RobotArm
	    PRIVATE
	    dl
	    -L${SDL2_LIBDIR} -lSDL2 -lSDL2_ttf
	    -lpng
	    )
endif()

target_link_libraries(RobotArm PRIVATE
    ${Boost_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

if(WIN32)
	target_link_libraries(RobotArm
	    PRIVATE
	    ${MKL_LIB_DIR}/mkl_core.lib
	    ${MKL_LIB_DIR}/mkl_intel_lp64.lib
	    ${MKL_LIB_DIR}/mkl_sequential.lib
	)
else()
	target_link_libraries(RobotArm
	    PRIVATE
	    -Wl,--start-group
	    ${MKL_LIB_DIR}/libmkl_core.a
	    ${MKL_LIB_DIR}/libmkl_intel_lp64.a
	    ${MKL_LIB_DIR}/libmkl_sequential.a
	    -Wl,--end-group
	    )
endif()


