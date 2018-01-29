cmake_minimum_required(VERSION 2.8.3)
project(sipi_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  apriltags_ros
  swarmie_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(PickUpController src/PickUpController.cpp)
add_library(DropOffController src/DropOffController.cpp)
add_library(SearchController src/SearchController.cpp)
add_library(DrivingController src/DrivingController.cpp)
add_library(GripperController src/GripperController.cpp)
add_library(FindHomeController src/FindHomeController.cpp)
add_library(VisualLocalization src/VisualLocalization.cpp)
add_library(ObstacleController src/ObstacleController.cpp)
add_library(AvoidHome src/AvoidHome.cpp)
add_library(targetFunctions src/targetFunctions.cpp)
add_library(localization src/Localization.cpp)

add_executable(sipi_controller src/sipi_controller.cpp)
target_link_libraries(sipi_controller
  PickUpController
  DropOffController
  SearchController
  DrivingController
  GripperController
  FindHomeController
  VisualLocalization
  ObstacleController
  AvoidHome
  targetFunctions
	localization
  ${catkin_LIBRARIES})

add_executable(visual_localization_node src/visual_localization_node.cpp)
target_link_libraries(visual_localization_node 
  VisualLocalization 
  ${catkin_LIBRARIES})

add_dependencies(sipi_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(localization ${catkin_EXPORTED_TARGETS})
add_dependencies(visual_localization_node ${catkin_EXPORTED_TARGETS})

add_executable(abridge src/abridge.cpp)
add_library(usbSerial src/usbSerial.cpp)

target_link_libraries( abridge usbSerial ${catkin_LIBRARIES})
