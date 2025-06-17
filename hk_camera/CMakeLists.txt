cmake_minimum_required(VERSION 3.0.2)
project(hk_camera)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        pluginlib
        image_transport
        cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES camera_manager_nodelet
        CATKIN_DEPENDS roscpp nodelet pluginlib image_transport cv_bridge sensor_msgs std_msgs
        DEPENDS OpenCV
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

link_directories(
        /opt/MVS/lib/64/
)

add_executable(hk_camera_node
        src/hk_camera_ros_node.cpp
        src/camera_manager.cpp
)
target_link_libraries(hk_camera_node
        ${catkin_LIBRARIES}
        MvCameraControl
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