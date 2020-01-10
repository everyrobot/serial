cmake_minimum_required(VERSION 2.8.3)
project(serial_example)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  er_ti_f28069m_drv8305
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    std_msgs 
LIBRARIES
    serial_example
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)


add_library(serial_example
    include/${PROJECT_NAME}/serial_example.h
    src/serial_example.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(serial_example
        ${er_ti_f28069m_drv8305_LIBRARIES}
        ${serial_example_LIBRARIES}
	${catkin_LIBRARIES})

add_executable(serial_example_node src/serial_example_node.cpp)
add_dependencies(serial_example_node serial_example)

target_link_libraries(serial_example_node
    serial_example
  ${catkin_LIBRARIES}
)


