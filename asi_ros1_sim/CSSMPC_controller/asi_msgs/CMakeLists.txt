cmake_minimum_required(VERSION 2.8.3)
project(asi_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
)


add_message_files(
  DIRECTORY msg
  FILES
  
  AsiTBSG.msg
  AsiClothoid.msg
  AsiClothoidPath.msg
  mapCA.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package()
