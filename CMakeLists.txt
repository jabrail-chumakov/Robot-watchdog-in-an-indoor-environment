## Init project.
cmake_minimum_required(VERSION 2.8.3)
project(robot_watchdog_in_an_indoor_environment)

## Find dependences.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  message_generation
)

## Uncomment if the package has a setup.py
catkin_python_setup()

## Generate messages in the 'msg' folder.
add_message_files(
  FILES
  Point.msg
)

## Generate services in the 'srv' folder.
add_service_files(
  FILES
  GetPose.srv
  SetPose.srv
  Battery.srv
)

## Generate actions in the 'action' folder.
add_action_files(
  FILES
  Control.action
  Plan.action
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

## The catkin_package macro generates cmake config files for your package.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arch_skeletton
   CATKIN_DEPENDS message_runtime # rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Define runnables.
catkin_install_python(
  PROGRAMS 
    scripts/planner.py
    scripts/controller.py
    scripts/robot_states.py
    scripts/fsm.py
    scripts/planner_helper.py
    scripts/controller_helper.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
