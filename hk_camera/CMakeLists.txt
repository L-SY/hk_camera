cmake_minimum_required(VERSION 3.0.2)
project(hk_camera)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

link_directories(
        /home/lsy/Downloads/MVS_STD_V3.0.1_241128/MVS-3.0.1_x86_64_20241128/MVS/lib/64         # 替换为你的SDK库路径
)

add_executable(test_camera
        test/test_camera.cpp
        src/camera_manager.cpp)
target_link_libraries(test_camera
        ${catkin_LIBRARIES}
        MvCameraControl
        pthread
        opencv_core
        opencv_highgui)

add_executable(test_parameter
        test/test_parameter.cpp
        src/camera_manager.cpp)
target_link_libraries(test_parameter
        ${catkin_LIBRARIES}
        MvCameraControl
        pthread
        opencv_core
        opencv_highgui)