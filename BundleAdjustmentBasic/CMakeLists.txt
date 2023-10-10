cmake_minimum_required(VERSION 3.17)
project(BundleAdjustmentBasic)

set(CMAKE_CXX_STANDARD 14)

#添加CMAKE_MODULE_PATH路径
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# OpenCV
find_package(OpenCV 3 REQUIRED)
# g2o
find_package(G2O REQUIRED)
# sophus
find_package(Sophus REQUIRED)


include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)

add_executable(BundleAdjustmentBasic main.cpp)
target_link_libraries(BundleAdjustmentBasic ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
