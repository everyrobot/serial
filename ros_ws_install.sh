#!/usr/bin/env bash
mkdir serial_control_ws 
cd serial_control_ws 
mkdir src   
cd src 
git clone https://github.com/everyrobot/serial-ros.git
git clone --branch er_drv_ros https://github.com/everyrobot/serial-ros.git
cd ..
catkin_make
source devel/setup.bash
gnome-terminal -x sh -c "roscore"
sleep 1.5
rosrun serial_example serial_example_node
