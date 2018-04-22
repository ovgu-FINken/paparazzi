cmake_minimum_required(VERSION 2.8.3)
project(camera_pylon)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++11 -Wno-literal-suffix")

#set(CMAKE_VERBOSE_MAKEFILE ON)

set(PYLON_DIR ${PROJECT_SOURCE_DIR}/pylon5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  image_transport
  camera_info_manager
  driver_base
  dynamic_reconfigure
)
find_package(OpenCV)
generate_dynamic_reconfigure_options(cfg/Camera.cfg)

catkin_package(
    CATKIN_DEPENDS image_transport cv_bridge camera_info_manager sensor_msgs dynamic_reconfigure

)

include_directories(
  ${PROJECT_SOURCE_DIR}/cfg/cpp
  ${PROJECT_SOURCE_DIR}/include
  ${PYLON_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PYLON_DIR}/lib64)

add_library(cam_node src/cam_node.cpp)
add_dependencies(cam_node ${PROJECT_NAME}_gencfg)


target_link_libraries(cam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pylonbase
  pylonutility
  GenApi_gcc_v3_0_Basler_pylon_v5_0
  GCBase_gcc_v3_0_Basler_pylon_v5_0
  Log_gcc_v3_0_Basler_pylon_v5_0
  MathParser_gcc_v3_0_Basler_pylon_v5_0
  XmlParser_gcc_v3_0_Basler_pylon_v5_0
  log4cpp_gcc_v3_0_Basler_pylon_v5_0
)
