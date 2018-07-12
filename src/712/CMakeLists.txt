cmake_minimum_required(VERSION 2.8.3)
project(ardrone_test)


add_compile_options(-std=c++11)

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


	
#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
	
#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 
#并且把该目录设为连接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)
	
#设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)
	
#增加子文件夹，也就是进入源代码文件夹继续构建
ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src)


add_executable(ardrone src/ardrone.cpp)
target_link_libraries(ardrone ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ardrone_detect src/ardrone_detect.cpp)
target_link_libraries(ardrone_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(Detection src/Detection.cpp)
target_link_libraries(Detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ChArUco_Detection src/ChArUco_Detection.cpp)
target_link_libraries(ChArUco_Detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})











