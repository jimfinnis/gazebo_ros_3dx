cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_3dx)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs tf urdf)

catkin_package(
    CATKIN_DEPENDS tf urdf geometry_msgs sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(publisher src/publisher.cc)
target_link_libraries(publisher ${catkin_LIBRARIES})

install(TARGETS publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(publisher3dx src/publisher3dx.cc)
target_link_libraries(publisher3dx ${catkin_LIBRARIES})

install(TARGETS publisher publisher3dx
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
   launch defs meshes
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
