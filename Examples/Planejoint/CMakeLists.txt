cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Planejoint main.cpp)

set_target_properties(Planejoint PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(Planejoint PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(Planejoint PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(Planejoint PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(Planejoint Springhead::Springhead)
target_link_libraries(Planejoint dimp)
target_link_libraries(Planejoint Scenebuilder)
target_link_libraries(Planejoint           ${EXPAT_LIBRARIES})
target_link_libraries(Planejoint           ${GLUT_LIBRARIES})
target_link_libraries(Planejoint           ${GLEW_LIBRARIES})
target_link_libraries(Planejoint           ${OPENGL_LIBRARIES})
target_link_libraries(Planejoint           ${Boost_LIBRARIES})
target_link_libraries(Planejoint           ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(Planejoint           ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(Planejoint           ${MKL_LIB_DIR}/mkl_sequential.lib)
