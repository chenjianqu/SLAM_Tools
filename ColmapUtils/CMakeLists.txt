cmake_minimum_required(VERSION 3.25)
project(ColmapUtils)

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

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})


set(THIRTY_LISB
        ${OpenCV_LIBS}
        ${spdlog_LIBRARIES}
        -lglog
        ${GFLAGS_LIBRARIES}
        yaml-cpp
        )


add_executable(colmap_select_keyframe_by_gps src/select_keyframe_by_gps.cpp src/utils/gps_pose.cpp src/utils/file_utils.cpp)
target_link_libraries(colmap_select_keyframe_by_gps ${THIRTY_LISB} )

add_executable(colmap_select_mask_by_rgb src/select_mask_by_rgb.cpp src/utils/gps_pose.cpp src/utils/file_utils.cpp)
target_link_libraries(colmap_select_mask_by_rgb  ${THIRTY_LISB} )


add_executable(colmap_yaml_setter src/yaml_setter.cpp )
target_link_libraries(colmap_yaml_setter  ${THIRTY_LISB} )


install(TARGETS  colmap_yaml_setter
        EXPORT colmap_utils_targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include
        )

install(
        EXPORT colmap_utils_targets
        FILE ${PROJECT_NAME}-config.cmake
        DESTINATION share/cmake/${PROJECT_NAME}/)

