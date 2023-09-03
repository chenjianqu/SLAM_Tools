cmake_minimum_required(VERSION 3.25)
project(AlignTrajectory)

set(CMAKE_CXX_STANDARD 17)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

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

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(THIRTY_LIBS
        ${GFLAGS_LIBRARIES}
        -lglog
        ${spdlog_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${CERES_LIBRARIES}
        )

include_directories(src)

add_executable(AlignTrajectory src/exe/align_trajectory.cpp src/utils.cpp)
target_link_libraries(AlignTrajectory ${THIRTY_LIBS})

add_executable(rotate_poses src/exe/rotate_poses.cpp src/utils.cpp)
target_link_libraries(rotate_poses ${THIRTY_LIBS})

add_executable(complete_trajectory src/exe/complete_trajectory.cpp src/utils.cpp  src/factor/pose_factor.cpp
        src/factor/eigenquaternion_localparameterization.h
        src/factor/eigenquaternion_localparameterization.cpp
        src/factor/math_util.hpp)
target_link_libraries(complete_trajectory ${THIRTY_LIBS} )

add_executable(complete_trajectory_propagate src/exe/complete_trajectory_propagate.cpp src/utils.cpp
        src/factor/math_util.hpp )
target_link_libraries(complete_trajectory_propagate ${THIRTY_LIBS})

add_executable(clip_timestamp src/exe/clip_timestamp.cpp src/utils.cpp)
target_link_libraries(clip_timestamp ${THIRTY_LIBS})
