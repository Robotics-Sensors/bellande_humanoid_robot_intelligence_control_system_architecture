cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_intelligence_control_system_read_write_demo)


if($ENV{ROS_VERSION} EQUAL 1)
  find_package(
    catkin REQUIRED COMPONENTS
    humanoid_robot_intelligence_control_system_controller_msgs
    roscpp
    sensor_msgs
    std_msgs
  )
else()
  find_package(ament_cmake REQUIRED)
endif()


if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
    roscpp
    humanoid_robot_intelligence_control_system_controller_msgs
    roscpp
    sensor_msgs
    std_msgs
  )
endif()


include_directories(${catkin_INCLUDE_DIRS})

add_executable(
  read_write
  src/read_write.cpp
)

add_dependencies(
  read_write
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  read_write
  ${catkin_LIBRARIES}
)

install(
  TARGETS read_write
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
