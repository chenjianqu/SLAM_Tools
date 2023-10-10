cmake_minimum_required(VERSION 3.25)
project(CoordinateSystemUtils)

set(CMAKE_CXX_STANDARD 17)


add_library(GeographicLib SHARED
        src/GeographicLib/Constants.hpp
        src/GeographicLib/Geocentric.hpp
        src/GeographicLib/Geocentric.cpp
        src/GeographicLib/LocalCartesian.hpp
        src/GeographicLib/LocalCartesian.cpp
         )



add_executable(CoordinateSystemUtils src/main.cpp  )
target_link_libraries(CoordinateSystemUtils GeographicLib)
