cmake_minimum_required(VERSION 2.8.3)
project(ctrl_cmd_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  ackermann_msgs
  autoware_msgs
)

catkin_package(
# INCLUDE_DIRS include
 CATKIN_DEPENDS
 roscpp
 geometry_msgs
 ackermann_msgs
 autoware_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ctrl_cmd_converter_node src/converter.cpp)
add_dependencies(ctrl_cmd_converter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ctrl_cmd_converter_node ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS ctrl_cmd_converter_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
