cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Slider main.cpp)

set_target_properties(Slider PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(Slider PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(Slider PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(Slider PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(Slider Springhead::Springhead)
target_link_libraries(Slider dimp)
target_link_libraries(Slider Scenebuilder)
target_link_libraries(Slider           ${EXPAT_LIBRARIES})
target_link_libraries(Slider           ${GLUT_LIBRARIES})
target_link_libraries(Slider           ${GLEW_LIBRARIES})
target_link_libraries(Slider           ${OPENGL_LIBRARIES})
target_link_libraries(Slider           ${Boost_LIBRARIES})
target_link_libraries(Slider           ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(Slider           ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(Slider           ${MKL_LIB_DIR}/mkl_sequential.lib)

