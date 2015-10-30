cmake_minimum_required(VERSION 2.8.3)
project(nord_pointcloud)
#set(CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")

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
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(calibration src/calibration.cpp)
add_executable(process src/process.cpp)
add_executable(objectdetection src/objectdetection.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(objectdetection ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(process ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
