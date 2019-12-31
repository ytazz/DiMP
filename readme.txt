[ about DiMP ]
DiMP is a C++ library for motion planning of robotic systems.
It uses gradient based technique for trajectory optimization.
Note that it is different from search-based motion planners such as RRT.

[ dependency ]
DiMP is dependent on some other libraries.
- Springhead
  github.com/sprphys/Springhead
  see description there.
  
- Scenebuilder
  github.com/ytazz/Scenebuilder
  see description there.

Some example programs included in DiMP are dependent on some other libraries.
- glwin
  github.com/ytazz/glwin
  see description there.

[ how to build ]
Use CMake. DiMP is mainly developed and tested on Windows + Visual Studio environment.
There should be not major problem in building it on linux, but one should be careful about some platform-dependent
 differences such as MKL and Lapack. 
Our current cmake files do not fully support linux + gcc environment.

- Open CMake-gui, choose DiMP's top directory as source directory and create and specify your 
  build directory (e.g. DiMP/build).
- Set the following variables.
 - If you are building DiMP library only:
  SPRINGHEAD_DIR            top directory of Springhead (e.g. c:\Springhead)
  SCENEBUILDER_INCLUDE_DIR  include directory of Scenebuilder
    if you have built and installed Scenebuilder using CMake, this is
    ${CMAKE_INSTALL_PREFIX}/include/Scenebuilder
  CMAKE_INSTALL_PREFIX      where you would like to install DiMP.
    recommended to set the same path as the other libraries' install directory.
  BUILD_EXAMPLES            check if you want to build minimum example programs
  BUILD_PROGRAMS            check if you want to build programs that use more complex models
  
* note:
  To build examples and programs, you first need to build and install the main library;
   do not check BUILD_EXAMPLES or BUILD_PROGRAMS it it is your first build.

 - If you checked BUILD_EXAMPLES
                                                       typical setting
  - DIMP_LIBRARY_DEBUG                                 ${CMAKE_INSTALL_PREFIX}/lib/dimpd.lib
  - DIMP_LIBRARY_RELEASE                               ${CMAKE_INSTALL_PREFIX}/lib/dimp.lib
  - SCENEBUILDER_LIBRARY_DEBUG                         ${CMAKE_INSTALL_PREFIX}/lib/scenebuilderd.lib
  - SCENEBUILDER_LIBRARY_RELEASE                       ${CMAKE_INSTALL_PREFIX}/lib/scenebuilder.lib
  - SPRINGHEAD_LIBRARY_DEBUG                           ${SPRINGHEAD_DIR}/generated/lib/[platform]/Springheadxx.xD[x64].lib
  - SPRINGHEAD_LIBRARY_RELEASE                         ${SPRINGHEAD_DIR}/generated/lib/[platform]/Springheadxx.x[x64].lib
  - MKL_INCLUDE_DIR                                    C:\Program Files (x86)\IntelSWTools\compilers_and_libraries_2017.2.187\windows\mkl\include
  - MKL_LIB_DIR                                        C:\Program Files (x86)\IntelSWTools\compilers_and_libraries_2017.2.187\windows\mkl\lib\intel64_win

 - If you checked BUILD_PROGRAMS
                                                       typical setting
  - GLWIN_INCLUDE_DIR                                  ${CMAKE_INSTALL_PREFIX}/include
  - GLWIN_LIBRARY_DEBUG                                ${CMAKE_INSTALL_PREFIX}/lib/glwind.lib
  - GLWIN_LIBRARY_RELEASE                              ${CMAKE_INSTALL_PREFIX}/lib/glwin.lib
  - SCENEBUILDER_DIMP_LIBRARY_DEBUG                    ${CMAKE_INSTALL_PREFIX}/lib/sbdimpd.lib
  - SCENEBUILDER_DIMP_LIBRARY_RELEASE                  ${CMAKE_INSTALL_PREFIX}/lib/sbdimp.lib
  - SCENEBUILDER_SPRGRAPHICS_LIBRARY_DEBUG             ${CMAKE_INSTALL_PREFIX}/lib/sbsprgraphicsd.lib
  - SCENEBUILDER_SPRGRAPHICS_LIBRARY_RELEASE           ${CMAKE_INSTALL_PREFIX}/lib/sbsprgraphics.lib
  - SDL2_INCLUDE_DIR                                   ${CMAKE_INSTALL_PREFIX}/include/SDL2
  - SDL2_LIBRARY_DEBUG                                 ${CMAKE_INSTALL_PREFIX}/lib/SDL2d.lib
  - SDL2_LIBRARY_RELEASE                               ${CMAKE_INSTALL_PREFIX}/lib/SDL2.lib
  - SDL2_ttf_INCLUDE_DIR                               ${CMAKE_INSTALL_PREFIX}/include/SDL2
  - SDL2_ttf_LIBRARY_DEBUG                             ${CMAKE_INSTALL_PREFIX}/lib/SDL2_ttfd.lib
  - SDL2_ttf_LIBRARY_RELEASE                           ${CMAKE_INSTALL_PREFIX}/lib/SDL2_ttf.lib
 
