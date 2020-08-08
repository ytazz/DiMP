cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Contact main.cpp)

set_target_properties(Contact PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(Contact PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(Contact PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(Contact PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(Contact debug     ${SPRINGHEAD_LIBRARY_DEBUG})
target_link_libraries(Contact optimized ${SPRINGHEAD_LIBRARY_RELEASE})
target_link_libraries(Contact dimp)
target_link_libraries(Contact Scenebuilder)
target_link_libraries(Contact           ${EXPAT_LIBRARIES})
target_link_libraries(Contact           ${GLUT_LIBRARIES})
target_link_libraries(Contact           ${GLEW_LIBRARIES})
target_link_libraries(Contact           ${OPENGL_LIBRARIES})
target_link_libraries(Contact           ${Boost_LIBRARIES})
target_link_libraries(Contact           ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(Contact           ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(Contact           ${MKL_LIB_DIR}/mkl_sequential.lib)
