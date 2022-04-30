cmake_minimum_required(VERSION 3.20)
project(VIODE_Test)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3)
include_directories( ${EIGEN3_INCLUDE_DIR})

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")


find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")


add_executable(VIODE_Test main.cpp parameters.cpp utils.cpp viode_utils.cpp)
target_link_libraries(VIODE_Test  ${OpenCV_LIBS} ${spdlog_LIBRARIES})
