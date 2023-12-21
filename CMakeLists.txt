cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_cluster_detect)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    auvlab_msgs
    nmea_msgs
    geometry_msgs
    message_generation
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    cv_bridge
)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    RadarSpoke.msg
    Centroids.msg
    RadarSegment.msg
    CloudClusterArray.msg
    CloudCluster.msg
    BoundingBox.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS cv_bridge geometry_msgs message_runtime pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf nmea_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_euclidean_cluster_detect
    src/pointcloud_euclidean_cluster_detect.cpp
    src/cluster.cpp)
add_executable(pointcloud_sweep_aggregator
    src/pointcloud_sweep_aggregator.cpp)

target_link_libraries(pointcloud_euclidean_cluster_detect ${catkin_LIBRARIES})
target_link_libraries(pointcloud_sweep_aggregator ${catkin_LIBRARIES})
