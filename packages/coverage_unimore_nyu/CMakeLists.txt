################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(coverage_unimore_nyu)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
#set(ARPL false)
ifind_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  #gazebo_ros
  message_generation
  fow_control
  pid_control
)
find_package(OsqpEigen REQUIRED)
#find_package(gazebo REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

# Generate custom msgs in msg folder
add_message_files(
  FILES
  PoseVector.msg
)

# Generate added messages dependancies
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES PIDController
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 message_runtime #gazebo_ros
  DEPENDS gazebo_ros
)


################################################################################
# Build
################################################################################
#link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${fow_control_INCLUDE_DIRS}
  ${pid_control_INCLUDE_DIRS}
  #${GAZEBO_INCLUDE_DIRS}
)

set(SFML_libraries
  "sfml-graphics"
  "sfml-window"
  "sfml-system"
)

add_executable(coverage_uav src/coverage_uav.cpp)
add_dependencies(coverage_uav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_uav ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${SFML_libraries})

# add_library(PIDController
#    src/PIDController.cpp
# )
# add_dependencies(PIDController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(FowController 
#   ${catkin_LIBRARIES}
#   OsqpEigen::OsqpEigen
# )

add_executable(coverage_distributed_supervisor src/coverage_distributed_supervisor.cpp)
add_dependencies(coverage_distributed_supervisor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_distributed_supervisor ${catkin_LIBRARIES}) #${GAZEBO_LIBRARIES}) #${SFML_libraries})


add_executable(coverage_distributed_robotnode src/coverage_distributed_robotnode.cpp)
add_dependencies(coverage_distributed_robotnode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_distributed_robotnode ${catkin_LIBRARIES} ${fow_control_LIBRARIES} OsqpEigen::OsqpEigen) #${GAZEBO_LIBRARIES}) #${SFML_libraries})

add_executable(checker src/check.cpp)
add_dependencies(checker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(checker ${catkin_LIBRARIES} ${fow_control_LIBRARIES} ${pid_control_LIBRARIES} OsqpEigen::OsqpEigen) #${GAZEBO_LIBRARIES}) #${SFML_libraries})

add_executable(recovery src/recovery.cpp)
add_dependencies(recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(recovery ${catkin_LIBRARIES} ${fow_control_LIBRARIES} ${pid_control_LIBRARIES} OsqpEigen::OsqpEigen) #${GAZEBO_LIBRARIES}) #${SFML_libraries})

add_executable(heading src/heading.cpp)
add_dependencies(heading ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(heading ${catkin_LIBRARIES} ${fow_control_LIBRARIES} ${pid_control_LIBRARIES} OsqpEigen::OsqpEigen) #${GAZEBO_LIBRARIES}) #${SFML_libraries})

add_executable(coverage_distributed_supervisor_id src/coverage_distributed_supervisor_id.cpp)
add_dependencies(coverage_distributed_supervisor_id ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_distributed_supervisor_id ${catkin_LIBRARIES}) #${GAZEBO_LIBRARIES}) #${SFML_libraries})

add_executable(coverage_supervisor_global src/coverage_supervisor_global.cpp)
add_dependencies(coverage_supervisor_global ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_supervisor_global ${catkin_LIBRARIES}) #${GAZEBO_LIBRARIES}) #${SFML_libraries})

add_executable(coverage_centralized src/coverage_centralized.cpp)
add_dependencies(coverage_centralized ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_centralized ${catkin_LIBRARIES} OsqpEigen::OsqpEigen) #${GAZEBO_LIBRARIES}) #${SFML_libraries})

################################################################################
# Install
################################################################################
#install(TARGETS coverage_uav
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
