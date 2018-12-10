cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(Biped main.cpp)
target_link_libraries(Biped dimp)
target_link_libraries(Biped Scenebuilder)
target_link_libraries(Biped Springhead)
target_link_libraries(Biped dl)
target_link_libraries(Biped ${EXPAT_LIBRARIES})
target_link_libraries(Biped ${LAPACK_LIBRARIES})
target_link_libraries(Biped lapacke)
target_link_libraries(Biped ${GLUT_LIBRARIES})
target_link_libraries(Biped ${GLEW_LIBRARIES})
target_link_libraries(Biped ${OPENGL_LIBRARIES})
target_link_libraries(Biped ${Boost_LIBRARIES})

