cmake_minimum_required(VERSION 2.8.3)
project(ardrone_test)


find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
)

find_package(OpenCV)

catkin_package()

include_directories(

  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ardrone src/ardrone.cpp)
target_link_libraries(ardrone ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ardrone_detect src/ardrone_detect.cpp)
target_link_libraries(ardrone_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(Detection src/Detection.cpp)
target_link_libraries(Detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})









