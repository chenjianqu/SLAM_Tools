cmake_minimum_required(VERSION 3.25)
project(ImuPreintegration)

set(CMAKE_CXX_STANDARD 17)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


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

aux_source_directory(src src_files)
aux_source_directory(src/utils src_utils_files)


add_executable(ImuPreintegration ${src_files} ${src_utils_files})
target_link_libraries(ImuPreintegration  ${OpenCV_LIBS} ${spdlog_LIBRARIES} )
