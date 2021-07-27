cmake_minimum_required(VERSION 2.8.3)
project(rgbd_image_extractor_package)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  cv_bridge
  ## geometry_msgs
  ## nav_msgs  
  ## tf
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ## Add dependencies of other libraries
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs ## geometry_msgs nav_msgs tf 
   DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Definition of libraries
## RGBDExtractorNode 
add_library(RGBDExtractor
  src/RGBDExtractorNode.cpp
)
target_link_libraries(RGBDExtractor
  ${catkin_LIBRARIES}
)
add_dependencies(RGBDExtractor
  ${catkin_EXPORTED_TARGETS}
)

## Definition of executables
## ros_cpp_template
add_executable(rgbd_image_extractor
  src/RGBDExtractor_main.cpp
)
target_link_libraries(rgbd_image_extractor
  RGBDExtractor
  ${catkin_LIBRARIES}
)

install(TARGETS rgbd_image_extractor

  RGBDExtractor

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

