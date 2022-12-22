cmake_minimum_required(VERSION 3.0.2)
project(sensor_cam_hough)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
)
find_package(OpenCV 4.5.5 REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  cam_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
