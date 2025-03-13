#ifndef __USB2CAN_ROS_H__
#define __USB2CAN_ROS_H__


#include <ros/ros.h>
#include <usb2can_ros/CANFrameMsg.h>
class USB2CAN_ROS_Node {
public:
    USB2CAN_ROS_Node(ros::NodeHandle& nh);

    ~USB2CAN_ROS_Node();

    void run();

private:
    ros::NodeHandle nh_;

    int handler_;

    ros::Subscriber sub_;

    ros::Publisher pub_;

    void callback(const usb2can_ros::CANFrameMsg::ConstPtr& msg);
};
#endif