cmake_minimum_required(VERSION 2.8.3)
project(sipi_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
  tf
  apriltags_ros
  swarmie_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  sipi_controller 
  src/PickUpController.cpp
  src/DropOffController.cpp
  src/SearchController.cpp
	src/DrivingController.cpp
	src/GripperController.cpp
	src/FindHomeController.cpp
	src/VisualLocalization.cpp
	src/ObstacleController.cpp
	src/AvoidHome.cpp
	src/targetFunctions.cpp
  src/mobility.cpp
)

add_executable(
	visual_localization_node
	src/visual_localization_node.cpp
	src/VisualLocalization.cpp
)
add_library(localization src/Localization.cpp)

add_dependencies(sipi_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(localization ${catkin_EXPORTED_TARGETS})
add_dependencies(visual_localization_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  sipi_controller
	localization
  ${catkin_LIBRARIES}
)
target_link_libraries(
  visual_localization_node
  ${catkin_LIBRARIES}
)

target_link_libraries( localization ${catkin_LIBRARIES})
