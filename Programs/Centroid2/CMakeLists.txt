cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Centroid2
    mpc.h
    mpc.cpp
    main.cpp)

set_target_properties(Centroid2 PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(Centroid2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(Centroid2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(Centroid2 PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(Centroid2 PRIVATE
    console
    Springhead::Springhead
    dimp
    Scenebuilder
    SDL2::SDL2
    SDL2_ttf::SDL2_ttf
    GLUT::GLUT
    GLEW::GLEW
    ${EXPAT_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${Boost_LIBRARIES}
    )

if(WIN32)
	target_link_libraries(Centroid2
	    PRIVATE
	    ${MKL_LIB_DIR}/mkl_core.lib
	    ${MKL_LIB_DIR}/mkl_intel_lp64.lib
	    ${MKL_LIB_DIR}/mkl_sequential.lib
	    )
else()
	target_link_libraries(Centroid2
	    PRIVATE
	    -Wl,--start-group
	    ${MKL_LIB_DIR}/libmkl_intel_lp64.a
	    ${MKL_LIB_DIR}/libmkl_core.a
	    ${MKL_LIB_DIR}/libmkl_sequential.a
	    -Wl,--end-group
	)
endif()

if(UNIX)
	target_link_libraries(Centroid2
	    PRIVATE
	    ${CMAKE_DL_LIBS}
	    -lpng
	    )
endif()

