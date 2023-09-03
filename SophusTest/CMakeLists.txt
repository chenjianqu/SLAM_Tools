cmake_minimum_required(VERSION 3.20)
project(SophusTest)

set(CMAKE_CXX_STANDARD 17)


find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(SophusTest main.cpp)
target_link_libraries(SophusTest ${Sophus_LIBRARIES})
