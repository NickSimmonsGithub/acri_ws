cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(acri_gazebo)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  plugins/tracked_vehicle_plugin_ROS
  plugins/simple_tracked_vehicle_plugin_ROS/deps/opende/include
  plugins/simple_tracked_vehicle_plugin_ROS/deps/threadpool
  plugins/simple_tracked_vehicle_plugin_ROS/deps/opende/src
)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${rospy_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES track_velocity.msg)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS}
${GAZEBO_MSG_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build the SimpleTrackedVehiclePluginROS and TrackedVehiclePluginROS libraries 
add_library(TrackedVehiclePluginROS SHARED plugins/tracked_vehicle_plugin_ROS/TrackedVehiclePluginROS.cc)
target_link_libraries(TrackedVehiclePluginROS ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(SimpleTrackedVehiclePluginROS SHARED plugins/simple_tracked_vehicle_plugin_ROS/SimpleTrackedVehiclePluginROS.cc)
target_link_libraries(SimpleTrackedVehiclePluginROS ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

target_link_libraries(SimpleTrackedVehiclePluginROS TrackedVehiclePluginROS)
add_dependencies(SimpleTrackedVehiclePluginROS TrackedVehiclePluginROS)

# Mark executable python scripts for installation
catkin_install_python(PROGRAMS
  scripts/keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



