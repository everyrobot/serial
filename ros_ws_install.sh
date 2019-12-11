#!/usr/bin/env bash
mkdir serial_control_ws 
cd serial_control_ws 
mkdir src   
cd src 
git clone https://github.com/everyrobot/er_ti_f28069m_drv8305.git
git clone https://github.com/everyrobot/serial-example.git
cd serial-example
git fetch
git checkout -b linux_serial
git pull origin linux_serial
cd ../.. 
catkin_make
source devel/setup.bash
gnome-terminal -x sh -c "roscore"
sleep 1.5
rosrun serial_example serial_example_node
