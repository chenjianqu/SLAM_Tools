cmake_minimum_required(VERSION 3.17)
project(G2oBasic)

set(CMAKE_CXX_STANDARD 14)

#添加CMAKE_MODULE_PATH路径
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})


# Eigen
include_directories("/usr/include/eigen3")


add_executable(G2oBasic main.cpp)
target_link_libraries(G2oBasic ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})