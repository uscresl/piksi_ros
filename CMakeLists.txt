cmake_minimum_required(VERSION 2.8.3)
project(piksi_ros)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  message_generation
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/PiksiDriverConfig.cfg
)

add_message_files(
   DIRECTORY msg
   FILES
   CarrierPhase.msg
   Ephemeris.msg
   Obs.msg
   Observations.msg
   Sid.msg
   SignalStatus.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES piksi_driver
  CATKIN_DEPENDS diagnostic_updater nav_msgs rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
