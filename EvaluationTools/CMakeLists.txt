cmake_minimum_required(VERSION 3.25)
project(EvaluationTools)

set(CMAKE_CXX_STANDARD 17)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories( ${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV_INCLUDE_DIRS:  ${OpenCV_INCLUDE_DIRS}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")



add_executable(EvaluationTools src/main.cpp)

add_executable(TumFormatWriter src/TumFormatWriter.cpp)

add_executable(ReadColmapToTum src/ReadColmapToTum.cpp)
target_link_libraries(ReadColmapToTum  ${OpenCV_LIBS} ${spdlog_LIBRARIES} )



