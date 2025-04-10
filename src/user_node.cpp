#include <ros/ros.h>
#include <usb2can_ros/CANFrameMsg.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "USB2CAN_ROS_node");
    std::string device;
    ros::NodeHandle nh;
    nh.getParam(ros::this_node::getName() +"/device", device);
    ros::Publisher pub = nh.advertise<usb2can_ros::CANFrameMsg>(device+"/can_tx", 1000);

    usb2can_ros::CANFrameMsg frame;
    ros::Rate loop_rate(1);
    while(ros::ok){
        frame.channel = 2;
        frame.can_id = 0x123;
        frame.frame_type = 0;
        frame.data_length = 8;
        frame.data[0] = 0x01;
        frame.data[1] = 0x02;
        frame.data[2] = 0x03;
        frame.data[3] = 0x04;
        frame.data[4] = 0x05;
        frame.data[5] = 0x06;
        frame.data[6] = 0x07;
        frame.data[7] = 0x08;
        pub.publish(frame);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
