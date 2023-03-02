cmake_minimum_required(VERSION 3.10)
project(sarwapangrasa)

set(CMAKE_CXX_STANDARD 17)
include(FetchContent)
FetchContent_Declare(
  opencv
  URL https://github.com/opencv/opencv/archive/refs/tags/4.7.0.tar.gz
)
FetchContent_MakeAvailable(opencv)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    ganisakta
    usb_cam
    cv_bridge
    message_generation
    message_runtime
)

add_message_files(
  FILES
  Artag.msg
  Artags.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs 
  geometry_msgs
  sarwapangrasa
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES sarwapangrasa
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs ganisakta usb_cam cv_bridge message_generation message_runtime
#  DEPENDS system_lib
)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)

# ------- Make Library -------
set(SOURCES src/artag/artag.cpp)

add_library(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# ------- Make Library -------


add_executable(artag_node src/nodes/artag/artag_node.cpp)
target_link_libraries(artag_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})
add_dependencies(artag_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
