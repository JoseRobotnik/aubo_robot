cmake_minimum_required(VERSION 2.8.3)
project(aubo_joint_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Debug")

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########
find_library(libArmConnect
	NAMES ArmConnect
	PATHS ./lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(aubo_joint_controller_node src/aubo_joint_controller_node.cpp)

## Add cmake target dependencies of the executable
set(libOURAPI_LIBS ${libArmConnect})

## Specify libraries to link a library or executable target against
target_link_libraries(aubo_joint_controller_node ${catkin_LIBRARIES} ${libOURAPI_LIBS})
