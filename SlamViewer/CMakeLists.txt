cmake_minimum_required(VERSION 3.25)
project(SlamViewer)

set(CMAKE_CXX_STANDARD 17)



set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
message(STATUS ${Pangolin_LIBRARIES})

find_package(gflags REQUIRED)
include_directories(${gflags_INCLUDE_DIRS})



include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/src
        src
        include
)

set(THIRTY_LIBS
        ${OpenCV_LIBRARIES}
        ${spdlog_LIBRARIES}
        ${gflags_LIBRARIES}
        ${Pangolin_LIBRARIES}
        -lglog
        -llz4
        -lcrypto
        )


add_executable(SlamViewer src/main.cpp src/FrameDrawer.cc src/MapDrawer.cc src/Viewer.cc)
target_link_libraries(SlamViewer ${THIRTY_LIBS})
