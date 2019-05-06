cmake_minimum_required(VERSION 2.8.3)
project(grasp_set_skill_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
  geometry_msgs
)

#Custom Msgs
add_message_files(
        FILES
        GraspCandidate.msg
        GraspCandidateArr.msg
)


add_action_files(
  FILES
  GraspSet.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS actionlib_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
