cmake_minimum_required(VERSION 3.1)
project(aruco_detection VERSION 3.1.0 LANGUAGES C CXX)


#yorktaglidar
set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  std_srvs
  message_generation 
  std_msgs
)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})


find_package(PCL 1.3 REQUIRED)

find_package(OpenCV REQUIRED )
find_package(yaml-cpp REQUIRED)

include_directories(/usr/local/include/yaml-cpp)
include_directories(${PCL_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (tag_detection src/main.cpp)

target_link_libraries (tag_detection ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES} yaml-cpp)


