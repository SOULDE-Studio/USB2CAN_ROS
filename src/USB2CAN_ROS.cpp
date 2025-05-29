#include "USB2CAN_ROS.h"
#include "usb_can.h"




USB2CAN_ROS_Node::USB2CAN_ROS_Node(ros::NodeHandle &nh): nh_(nh){
    // read configuration
    std::string device_;

    nh_.getParam(ros::this_node::getName() +"/device", device_);

    handler_ = openUSBCAN(("/dev/" + device_).c_str());
    
    sub_ = nh_.subscribe(device_ +"/can_tx", 1000, &USB2CAN_ROS_Node::callback, this);
    pub_ = nh_.advertise<usb2can_ros::CANFrameMsg>(device_ +"/can_rx", 1000);

    ROS_INFO("USB2CAN_ROS Node is running");
    
    ROS_INFO("device: %s and handler: %d", device_.c_str(), handler_);

    if(handler_ < 0){
        ROS_ERROR("Failed to open USB2CAN device");
        exit(0);
    }

}
USB2CAN_ROS_Node::~USB2CAN_ROS_Node(){
    closeUSBCAN(handler_);
}


void USB2CAN_ROS_Node::run(){
    // start ROS loop with 100hz
    ros::Rate loop_rate(20000);
    usb2can_ros::CANFrameMsg frame;
    FrameInfo frame_info;
    uint8_t frame_data[8];

    while(ros::ok())
    {
        //读取一次USB2CAN数据
        int ret = readUSBCAN(handler_, &frame.channel, &frame_info, frame_data, 1000);
       
        //从订阅话题读取要发给USB2CAN的数据
        ros::spinOnce();
        
        if(ret < 0){
            continue;
        }
        frame.can_id = frame_info.canID;
        frame.frame_type = frame_info.frameType;
        frame.data_length = frame_info.dataLength;
        for(int i = 0; i < frame_info.dataLength; i++){
            frame.data[i] = frame_data[i];
        }
        pub_.publish(frame);
     
        
        loop_rate.sleep();
    }
}

void USB2CAN_ROS_Node::callback(const usb2can_ros::CANFrameMsg::ConstPtr& msg){
    //publish to USB2CAN
    FrameInfo frame_info;
    uint8_t frame_data[8];
    frame_info.canID = msg->can_id;
    frame_info.frameType = msg->frame_type;
    frame_info.dataLength = msg->data_length;
    for(int i = 0; i < msg->data_length; i++){
        frame_data[i] = msg->data[i];
    }

    int ret = sendUSBCAN(handler_, msg->channel, &frame_info, frame_data);
   
}


