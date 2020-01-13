uC side:
1. Flash master_test branch from https://github.com/everyrobot/er_ti_f28069m_drv8305/tree/master_test on f28069 

ROS side:
1. Download serial-ros from here at master branch https://github.com/everyrobot/serial-ros
2. Download er_ti_f28069m_drv8305 from here at er_drv_ros branch https://github.com/everyrobot/serial-ros/tree/er_drv_ros
3. compile and rosrun serial_example serial_example_node for tactile_bldc_serial functioning





HW interface:

    1. Download serial-ros from here at master branch https://github.com/everyrobot/serial-ros
    
    2. Download er_ti_f28069m_drv8305 at er_drv_ros branch https://github.com/everyrobot/serial-ros/tree/er_drv_ros
    
    3. Download rrbot_description from https://github.com/everyrobot/gazebo_ros_demos.git
    
    4. Download at er_dev branch https://github.com/everyrobot/ros_control_boilerplate.git
    
    5. compile and roslaunch ros_control_boilerplate rrbot_hardware.launch

RVIZ:

#include "geometry_msgs/WrenchStamped.h"

ros::Publisher tactile_force_pub;

tactile_force_pub = nh_.advertise<geometry_msgs::WrenchStamped>("tactile_pcb1_median", 1000);

geometry_msgs::WrenchStamped pcb1_msg;

            pcb1_msg.header.stamp = ros::Time::now();
            
            pcb1_msg.header.frame_id = "/map";
            
            pcb1_msg.wrench.force.z = 0;
            
            for (int i = 0; i < NO_OF_CHANNELS; i++)
            
                pcb1_msg.wrench.force.z += tactile_serial_read[i] / 35;

            tactile_force_pub.publish(pcb1_msg);
