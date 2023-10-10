cmake_minimum_required(VERSION 3.25)
project(PoseInterpolation)

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


add_executable(PoseInterpolation src/main.cpp src/file_utils.cpp src/tum_pose.cpp src/interpolator.cpp)
target_link_libraries(PoseInterpolation  ${OpenCV_LIBS} ${spdlog_LIBRARIES} )


add_executable(interpolation_by_txt src/main_interpolation_by_txt.cpp src/file_utils.cpp src/tum_pose.cpp src/interpolator.cpp)
target_link_libraries(interpolation_by_txt  ${OpenCV_LIBS} ${spdlog_LIBRARIES} )
