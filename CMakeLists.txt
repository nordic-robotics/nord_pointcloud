cmake_minimum_required(VERSION 2.8.3)
project(nord_pointcloud)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wno-unused-result ${CMAKE_CXX_FLAGS}")

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

find_package(HDF5)
find_package(FLANN)
include_directories(${FLANN_INCLUDE_DIRS})
include_directories(${HDF5_INCLUDE_DIR})


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
add_executable(debrisdetection src/debrisdetection.cpp)
add_executable(testClassification src/testClassification.cpp)
add_dependencies(testClassification ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testClassification ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})


add_dependencies(objectdetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(calibration ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(objectdetection ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(process ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

add_executable(cleverwallremoval src/cleverwallremoval.cpp)
add_dependencies(cleverwallremoval ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cleverwallremoval ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

add_executable(gatherdata src/gatherdata.cpp)
add_dependencies(gatherdata ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gatherdata ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

add_executable(flann src/flann.cpp)
add_dependencies(flann ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flann ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

add_executable(evenmorecleverwallremoval2 src/evenmorecleverwallremoval2.cpp)
add_dependencies(evenmorecleverwallremoval2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(evenmorecleverwallremoval2 ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

add_dependencies(debrisdetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(debrisdetection ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})


