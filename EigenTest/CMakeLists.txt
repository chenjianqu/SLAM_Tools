cmake_minimum_required(VERSION 3.20)
project(EigenTest)

set(CMAKE_CXX_STANDARD 17)


set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)



find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(EigenTest src/main.cpp)

add_executable(EulerToRotationMatrix src/EulerToRotationMatrix.cpp)

add_executable(RotationMatrixToEuler src/RotationMatrixToEuler.cpp)
