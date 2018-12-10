cmake_minimum_required(VERSION 2.8.12) # Default version on Ubuntu 14.04

add_executable(RobotArm
  main.cpp
  mymodule.cpp
  RobotArm.cpp
  Workspace.cpp
)
target_link_libraries(RobotArm console)
target_link_libraries(RobotArm glwin)
target_link_libraries(RobotArm sbdimp)
target_link_libraries(RobotArm sbsprgraphics)
target_link_libraries(RobotArm dimp)
target_link_libraries(RobotArm Scenebuilder)
target_link_libraries(RobotArm Springhead)
target_link_libraries(RobotArm dl)
target_link_libraries(RobotArm ${EXPAT_LIBRARIES})
target_link_libraries(RobotArm ${LAPACK_LIBRARIES})
target_link_libraries(RobotArm lapacke)
target_link_libraries(RobotArm ${GLUT_LIBRARIES})
target_link_libraries(RobotArm ${GLEW_LIBRARIES})
target_link_libraries(RobotArm ${OPENGL_LIBRARIES})
target_link_libraries(RobotArm ${Boost_LIBRARIES})
target_link_libraries(RobotArm -L${SDL2_LIBDIR} -lSDL2 -lSDL2_ttf)
target_link_libraries(RobotArm -lpng)
