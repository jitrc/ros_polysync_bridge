cmake_minimum_required(VERSION 2.8.3)
project(ros_polysync_bridge)
#cmake_policy(SET CMP0054 NEW)
include( polysync_build_res.cmake )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



 add_message_files(
   FILES
   IbeoObject2.msg
   IbeoObjectArray.msg
   laneModel.msg
   laneModelAll.msg
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PSYNC_INCLUDE_DIRS} 
) 

add_executable( ${PROJECT_NAME} src/ros_polysync_bridge_node.cpp )
add_dependencies(${PROJECT_NAME} ros_polysync_bridge_generate_messages_cpp)
target_link_libraries( ${PROJECT_NAME}   ${catkin_LIBRARIES} ${PSYNC_LIBS}  )
#target_link_libraries( ${PROJECT_NAME}   ${catkin_LIBRARIES}  )


