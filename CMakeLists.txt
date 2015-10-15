cmake_minimum_required(VERSION 2.8.3)
project(nord_pointcloud)

find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)


catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(calibration src/calibration.cpp)
add_executable(process src/process.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(process ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
