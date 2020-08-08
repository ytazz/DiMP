cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Balljoint main.cpp)

set_target_properties(Balljoint PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(Balljoint PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(Balljoint PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(Balljoint PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(Balljoint debug     ${SPRINGHEAD_LIBRARY_DEBUG})
target_link_libraries(Balljoint optimized ${SPRINGHEAD_LIBRARY_RELEASE})
target_link_libraries(Balljoint dimp)
target_link_libraries(Balljoint Scenebuilder)
target_link_libraries(Balljoint           ${EXPAT_LIBRARIES})
target_link_libraries(Balljoint           ${GLUT_LIBRARIES})
target_link_libraries(Balljoint           ${GLEW_LIBRARIES})
target_link_libraries(Balljoint           ${OPENGL_LIBRARIES})
target_link_libraries(Balljoint           ${Boost_LIBRARIES})
target_link_libraries(Balljoint           ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(Balljoint           ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(Balljoint           ${MKL_LIB_DIR}/mkl_sequential.lib)
