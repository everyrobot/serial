#include "serial_example/serial_example.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_example_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    uint16_t source_id = 1;
    uint16_t node_id = 100;
    std::string serial_port = "/dev/ttyUSB3";

    Serial serial(nh, source_id, node_id, serial_port);
    //ros::waitForShutdown();

    //ROS_INFO("shutdown node");
    return 0;
}
