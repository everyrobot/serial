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
    er_ti_f28069m_drv8305
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)



add_library(serial_example_lib src/serial_example.cpp)
add_dependencies(serial_example_lib er_ti_f28069m_drv8305)
target_link_libraries(serial_example_lib
	${er_ti_f28069m_drv8305_LIBRARIES}
	${catkin_LIBRARIES})



add_executable(serial_example_node src/serial_example_node.cpp)
add_dependencies(serial_example_node serial_example_lib)

target_link_libraries(serial_example_node
    serial_example_lib
  ${catkin_LIBRARIES}
)


