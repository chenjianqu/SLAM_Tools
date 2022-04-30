cmake_minimum_required(VERSION 3.17)
project(CeresBasic)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(CeresBasic main.cpp)
target_link_libraries(CeresBasic ${OpenCV_LIBS} ${CERES_LIBRARIES})
