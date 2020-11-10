cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Centroid main.cpp)

set_target_properties(Centroid PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(Centroid PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_INSTALL_DIR})
set_target_properties(Centroid PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_INSTALL_DIR})
set_target_properties(Centroid PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY    ${BIN_DIR})

target_link_libraries(Centroid console)
target_link_libraries(Centroid Springhead::Springhead)
target_link_libraries(Centroid dimp)
target_link_libraries(Centroid Scenebuilder)
target_link_libraries(Centroid SDL2::SDL2)
target_link_libraries(Centroid SDL2_ttf::SDL2_ttf)
target_link_libraries(Centroid GLEW::glew)
target_link_libraries(Centroid ${EXPAT_LIBRARIES})
target_link_libraries(Centroid ${LAPACK_LIBRARIES})
target_link_libraries(Centroid ${GLUT_LIBRARIES})
target_link_libraries(Centroid ${OPENGL_LIBRARIES})
target_link_libraries(Centroid ${Boost_LIBRARIES})
target_link_libraries(Centroid debug     ${PNG_LIBRARY_DEBUG})
target_link_libraries(Centroid optimized ${PNG_LIBRARY_RELEASE})
target_link_libraries(Centroid           ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(Centroid           ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(Centroid           ${MKL_LIB_DIR}/mkl_sequential.lib)

