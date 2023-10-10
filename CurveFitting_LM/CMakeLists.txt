cmake_minimum_required(VERSION 3.17)
project(CurveFitting_LM)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")


include_directories(${PROJECT_SOURCE_DIR}/include)



set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${GLOG_LIBRARIES}
        )


add_subdirectory(src)

add_executable(CurveFitting_LM main.cpp)
target_link_libraries(CurveFitting_LM mysolver ${THIRD_PARTY_LIBS})