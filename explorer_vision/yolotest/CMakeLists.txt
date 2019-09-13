cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_LIBRARY_PATH /opt/local/lib ${CMAKE_LIBRARY_PATH})
set(OpenCV_DIR ~/opencv-3.4.4/build)

project(yolotest)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_definitions(-std=c++11)
ADD_DEFINITIONS(-DOPENCV)
# 防止opencv报错

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
  hector_worldmodel_msgs
)

message(${OpenCV_INCLUDE_DIRS})

#这个是 yolo 动态库的路径
find_library(darknet libdarknet.so /home/szj/catkin_ws/src/nwpu_explorer/explorer_vision/yolotest/darknet)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yolotest
  CATKIN_DEPENDS hector_worldmodel_msgs cv_bridge image_transport  roscpp rospy sensor_msgs std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /home/szj/catkin_ws/src/nwpu_explorer/explorer_vision/yolotest/darknet/include # 修改绝对路径
)

aux_source_directory(./src/ DIR_SRC)

add_executable(yolotest_node src/yoloTest.cpp)
add_dependencies(yolotest_node hector_worldmodel_msgs_gencpp)

target_link_libraries(yolotest_node 
${catkin_LIBRARIES}
${OpenCV_LIBS} 
${darknet} 
/home/szj/catkin_ws/src/nwpu_explorer/explorer_vision/yolotest/darknet/libdarknet.so)