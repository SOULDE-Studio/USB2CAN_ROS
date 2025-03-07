#ifndef __USB2CAN_ROS_H__
#define __USB2CAN_ROS_H__


#include <ros/ros.h>
#include <USB2CAN_ROS/CANFrameMsg.h>
class USB2CAN_ROS_Node {
public:
    USB2CAN_ROS_Node(ros::NodeHandle& nh);

    void run();

private:
    ros::NodeHandle nh_;

    int handler_;

    ros::Subscriber sub_;

    ros::Publisher pub_;

    void callback(const USB2CAN_ROS::CANFrameMsg::ConstPtr& msg);
};
#endif