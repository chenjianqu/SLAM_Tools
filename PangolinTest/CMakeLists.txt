cmake_minimum_required(VERSION 3.26)
project(PangolinTest)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
message(STATUS ${Pangolin_LIBRARIES})

set(pangolin_lib
        pango_display
        pango_python
        pango_plot
)

add_executable(PangolinTest src/main.cpp)
target_link_libraries(PangolinTest ${pangolin_lib})

add_executable(pangolin_multi_thread_test src/pangolin_multi_thread_test.cpp)
target_link_libraries(pangolin_multi_thread_test ${pangolin_lib})

add_executable(pangolin_gui_test src/pangolin_gui_test.cpp)
target_link_libraries(pangolin_gui_test ${pangolin_lib})

add_executable(pangolin_image_test src/pangolin_image_test.cpp)
target_link_libraries(pangolin_image_test ${pangolin_lib} ${OpenCV_LIBRARIES})

add_executable(pangolin_draw_curve src/pangolin_draw_curve.cpp)
target_link_libraries(pangolin_draw_curve ${pangolin_lib} ${OpenCV_LIBRARIES})

add_executable(pangolin_draw_pose src/pangolin_draw_pose.cpp)
target_link_libraries(pangolin_draw_pose ${pangolin_lib} ${OpenCV_LIBRARIES})

add_executable(pangolin_slam_viewer src/pangolin_slam_viewer.cpp src/slam_visualizer.cpp)
target_link_libraries(pangolin_slam_viewer ${pangolin_lib} ${OpenCV_LIBRARIES})


