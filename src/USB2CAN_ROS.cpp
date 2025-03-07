#include "USB2CAN_ROS.h"
#include "usb_can.h"
USB2CAN_ROS_Node::USB2CAN_ROS_Node(ros::NodeHandle &nh): nh_(nh){
    // read configuration
    std::string device_;
    sub_ = nh_.subscribe("can_tx", 1000, &USB2CAN_ROS_Node::callback, this);
    pub_ = nh_.advertise<USB2CAN_ROS::CANFrameMsg>("can_rx", 1000);
    nh_.getParam("device", device_);
    handler_ = openUSBCAN(device_.c_str());
}
void USB2CAN_ROS_Node::run(){
    // start ROS loop with 100hz
    ros::Rate loop_rate(100);
    USB2CAN_ROS::CANFrameMsg frame;
    FrameInfo frame_info;
    uint8_t frame_data[8];
    while(ros::ok()){
        int ret = readUSBCAN(handler_, &frame.channel, &frame_info, frame_data, 10000);
        if(ret < 0){
            ROS_ERROR("readUSB2CAN timeout");
            continue;
        }
        frame.can_id = frame_info.canID;
        frame.frame_type = frame_info.frameType;
        frame.data_length = frame_info.dataLength;
        for(int i = 0; i < frame_info.dataLength; i++){
            frame.data[i] = frame_data[i];
        }
        pub_.publish(frame);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void USB2CAN_ROS_Node::callback(const USB2CAN_ROS::CANFrameMsg::ConstPtr& msg){
    //publish to USB2CAN
    FrameInfo frame_info;
    uint8_t frame_data[8];
    frame_info.canID = msg->can_id;
    frame_info.frameType = msg->frame_type;
    frame_info.dataLength = msg->data_length;
    for(int i = 0; i < msg->data_length; i++){
        frame_data[i] = msg->data[i];
    }

    sendUSBCAN(handler_, msg->channel, &frame_info, frame_data);
}