cmake_minimum_required(VERSION 2.8.3)
project(consai_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   robot_commands.msg
   robot_packet.msg
   RobotPoses.msg
   nodeData.msg
   nodeDataArray.msg
   VisionObservations.msg
   VisionPacket.msg
   VisionRobotPackets.msg
   AIStatus.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
