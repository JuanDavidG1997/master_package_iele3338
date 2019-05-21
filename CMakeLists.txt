cmake_minimum_required(VERSION 2.8.3)
project(master_package_iele3338)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(Qt5Widgets_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5Widgets")
set(Qt5Network_DIR "/usr/lib/x86_64-linux-gnu/cmake/Qt5Network")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  geometry_msgs
  master_msgs_iele3338
  Qt5Widgets
  Qt5Gui
  Qt5Core
  Qt5Network
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  master_msgs_iele3338
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy geometry_msgs message_runtime master_msgs_iele3338
   DEPENDS Qt5Widgets Qt5Gui Qt5Core Qt5Network
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

set(MASTER-PACKAGE-IELE3338-SRCS
  src/main.cpp
  src/master_window.cpp
  src/ros_thread.cpp
  src/plot_node_thread.cpp
)

set(MASTER-PACKAGE-IELE3338-HDRS
  include/master_package_iele3338/master_window.h
  include/master_package_iele3338/ros_thread.h
  include/master_package_iele3338/plot_node_thread.h
)

qt5_wrap_cpp(MASTER-PACKAGE-IELE3338-MOCS ${MASTER-PACKAGE-IELE3338-HDRS})

add_executable(master_node ${MASTER-PACKAGE-IELE3338-SRCS} ${MASTER-PACKAGE-IELE3338-HDRS})
add_dependencies(master_node ${master_package_iele3338_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(master_node Qt5::Widgets ${catkin_LIBRARIES})
