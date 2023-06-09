cmake_minimum_required(VERSION 3.0.2)
project(visual_servoing)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
## Generate messages in the 'msg' folder
#add_message_files(
 #  FILES
 #)
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   ctrl_srv_command.srv  
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visual_servoing
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vbmbot_motion.py
  #scripts/vbmbot_joint_controller.py
  #scripts/vbmbot_joint_controller_service.py
  #scripts/manager_client.py
   
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(vbm_image_feed src/image_feed.cpp)
target_link_libraries(vbm_image_feed ${catkin_LIBRARIES})