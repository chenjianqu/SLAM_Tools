cmake_minimum_required(VERSION 3.20)
project(TwoViewGeometryTest)

set(CMAKE_CXX_STANDARD 17)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIRS})
message("spdlog version:${spdlog_VERSION}")
message("spdlog dir:${spdlog_DIR}")


find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV DIR:  ${OpenCV_DIR}")
message("OpenCV OpenCV_VERSION:  ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

find_package(Boost REQUIRED COMPONENTS
        program_options
        filesystem
        graph
        system
        unit_test_framework)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})


find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})



include_directories( ${PROJECT_SOURCE_DIR}/src)


set(MyLibs
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${GLOG_LIBRARIES}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
        ${spdlog_LIBRARIES}
        -pthread
        )


aux_source_directory(src/base base_sources)
aux_source_directory(src/estimators estimators_sources)
aux_source_directory(src/feature feature_sources)
aux_source_directory(src/optim optim_sources)
aux_source_directory(src/util util_sources)


add_executable(TwoViewGeometryTest src/main.cpp ${base_sources} ${estimators_sources} ${feature_sources}
        ${optim_sources} ${util_sources})
target_link_libraries(TwoViewGeometryTest ${MyLibs})
