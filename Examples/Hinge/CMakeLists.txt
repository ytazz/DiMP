cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Hinge main.cpp)

set_target_properties(Hinge PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(Hinge PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(Hinge PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(Hinge PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(Hinge Springhead::Springhead)
target_link_libraries(Hinge dimp)
target_link_libraries(Hinge Scenebuilder)
target_link_libraries(Hinge           ${EXPAT_LIBRARIES})
target_link_libraries(Hinge           ${GLUT_LIBRARIES})
target_link_libraries(Hinge           ${GLEW_LIBRARIES})
target_link_libraries(Hinge           ${OPENGL_LIBRARIES})
target_link_libraries(Hinge           ${Boost_LIBRARIES})
target_link_libraries(Hinge           ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(Hinge           ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(Hinge           ${MKL_LIB_DIR}/mkl_sequential.lib)
