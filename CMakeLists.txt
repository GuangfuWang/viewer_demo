cmake_minimum_required(VERSION 3.10)
project(test)

#used for QT5
set(CMAKE_AUTOMOC  ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_CXX_STANDARD 17)
message(STATUS "Minimal C++ version required: c++14")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ltbb -mavx -march=native")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ltbb -mavx -march=native")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

link_directories(/usr/lib/x86_64-linux-gnu/)

option(__WITH_TBB "running with tbb library support" ON)
if (${__WITH_TBB})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ltbb")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ltbb")
    message(STATUS "You have add Tbb support")
endif ()

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    message(STATUS "DEBUG MODE: ON")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
else()
    message(STATUS "Release Mode: ON")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 ")
endif()

#find QT5 dependencies
find_package(Qt5 REQUIRED Qml Core Widgets Gui OpenGL)
include_directories(${Qt5_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()

include_directories("/usr/include/x86_64-linux-gnu/qt5/QtXml")

#find glew
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIR})

#find kinect driver dependencies
find_package(libfreenect REQUIRED)
include_directories(${FREENECT_INCLUDE_DIRS})

include_directories("/usr/include/ni/")

include_directories("/opt/intel/mkl/include")

#message(STATUS "This project is made by Guangfu WANG, from Tsinghua University. All rights reserved.")
#message(STATUS "Email: buaawgf@hotmail.com")

add_executable(test main.cpp
        viewer/gl_mesh/camera.h viewer/gl_mesh/camera.cpp
        viewer/gl_mesh/meshWindow.h viewer/gl_mesh/meshWindow.cpp
        viewer/gl_mesh/container.h viewer/gl_mesh/container.cpp
        viewer/gl_mesh/gl_include.hpp

        MarchingCubes.h MarchingCubes.cpp
        )


target_link_libraries(test
         libpthread.so
        ${OpenCV_LIBRARIES} tbb
        libGLEW.so libGL.so libQGLViewer.so
        ${FREENECT_LIBRARIES} ${QGLViewer}
        libglut.so libGLU.so
        Qt5::Core Qt5::Widgets Qt5::Gui Qt5::OpenGL)