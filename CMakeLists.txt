cmake_minimum_required(VERSION 2.8.3)
project(nord_pointcloud)
set(CMAKE_CXX_FLAGS "-O3 -Wall ${CMAKE_CXX_FLAGS}")

find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  roslib
  pcl_conversions
  pcl_ros
  nord_messages
)

catkin_package(
CATKIN_DEPENDS message_runtime nord_messages
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

add_dependencies(objectdetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(calibration ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(objectdetection ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(process ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

