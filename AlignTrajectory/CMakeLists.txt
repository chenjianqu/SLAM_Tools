cmake_minimum_required(VERSION 3.25)
project(AlignTrajectory)

set(CMAKE_CXX_STANDARD 17)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


add_definitions(-DSPDLOG_FMT_EXTERNAL)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(calib_manager REQUIRED)
include_directories(${calib_manager_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

set(THIRTY_LIBS
        ${GFLAGS_LIBRARIES}
        -lglog
        fmt
        ${spdlog_LIBRARIES}
        ${OpenCV_LIBRARIES}
        calib_manager
        ${Sophus_LIBRARIES}
        ${Ceres_LIBRARIES}
        -lceres
        )

include_directories(src)

add_executable(AlignTrajectory src/exe/align_trajectory.cpp src/utils.cpp)
target_link_libraries(AlignTrajectory ${THIRTY_LIBS})

add_executable(rotate_poses src/exe/rotate_poses.cpp src/utils.cpp)
target_link_libraries(rotate_poses ${THIRTY_LIBS})

add_executable(complete_trajectory src/exe/complete_trajectory.cpp src/utils.cpp  src/factor/pose_factor.cpp
        src/factor/local_parameterization.h
        src/factor/local_parameterization.cpp
        src/factor/math_util.hpp
        src/optimize.cpp
        src/pose.cpp
        )
target_link_libraries(complete_trajectory ${THIRTY_LIBS})

add_executable(complete_trajectory_propagate src/exe/complete_trajectory_propagate.cpp src/utils.cpp
        src/factor/math_util.hpp )
target_link_libraries(complete_trajectory_propagate ${THIRTY_LIBS})

add_executable(clip_timestamp src/exe/clip_timestamp.cpp src/utils.cpp)
target_link_libraries(clip_timestamp ${THIRTY_LIBS})
