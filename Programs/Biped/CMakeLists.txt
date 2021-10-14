cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Biped main.cpp)

set_target_properties(Biped PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(Biped PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(Biped PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(Biped PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(Biped console)
target_link_libraries(Biped Springhead::Springhead)
target_link_libraries(Biped dimp)
target_link_libraries(Biped Scenebuilder)
target_link_libraries(Biped SDL2::SDL2)
target_link_libraries(Biped SDL2_ttf::SDL2_ttf)
target_link_libraries(Biped GLUT::GLUT)
target_link_libraries(Biped GLEW::GLEW)
target_link_libraries(Biped ${EXPAT_LIBRARIES})
target_link_libraries(Biped ${LAPACK_LIBRARIES})
target_link_libraries(Biped ${OPENGL_LIBRARIES})

if(WIN32)
	target_link_libraries(Biped            ${MKL_LIB_DIR}/mkl_core.lib)
	target_link_libraries(Biped            ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
	target_link_libraries(Biped            ${MKL_LIB_DIR}/mkl_sequential.lib)
else()
	target_link_libraries(Biped            ${MKL_LIB_DIR}/libmkl_core.a)
	target_link_libraries(Biped            ${MKL_LIB_DIR}/libmkl_intel_lp64.a)
	target_link_libraries(Biped            ${MKL_LIB_DIR}/libmkl_sequential.a)
endif()

if(UNIX)
	target_link_libraries(Biped ${CMAKE_DL_LIBS})
	target_link_libraries(Biped -lpng)
endif()


