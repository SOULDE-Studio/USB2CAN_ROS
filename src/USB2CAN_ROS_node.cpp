#include <ros/ros.h>
#include "USB2CAN_ROS.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "USB2CAN_ROS_node");
    ros::NodeHandle nh;
    USB2CAN_ROS_Node usb2can(nh);
    usb2can.run();

    return 0;
}
