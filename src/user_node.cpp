#include <ros/ros.h>
#include <usb2can_ros/CANFrameMsg.h>

#define STANDARD 0
#define EXTENDED 1

int can1_rx_count=0;
int can1_tx_count=0;


void CANFrameMsgCallback(const usb2can_ros::CANFrameMsg::ConstPtr& msg){
    can1_rx_count++;
    if (can1_rx_count % 1000 == 0)
    {
        std::cout <<"    Send = "<<can1_tx_count <<"    Received CAN :" <<can1_rx_count <<"    loss rate : " <<100*(double)(can1_tx_count - can1_rx_count)/(double)can1_tx_count <<"%"<<std::endl; 
    }
    
    // ROS_INFO("Received CAN Frame: %d,channel: %d", msg->data_length, msg->channel);
    // for(int i = 0; i < msg->data_length; i++){
    //     ROS_INFO("Data[%d]: %d", i, msg->data[i]);
    // }
    // ROS_INFO("\r\n");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "USB2CAN_ROS_node");
    std::string device;
    ros::NodeHandle nh;
    nh.getParam(ros::this_node::getName() +"/device", device);

    ros::Publisher pub = nh.advertise<usb2can_ros::CANFrameMsg>(device+"/can_tx", 1000);

    ros::Subscriber sub = nh.subscribe(device+"/can_rx", 1000, &CANFrameMsgCallback);

    usb2can_ros::CANFrameMsg frame;
    ros::Rate loop_rate(8000);
    while(ros::ok){
        can1_tx_count++;

        frame.channel = 2; //使用CAN2发送
        frame.can_id = 0x123;
        frame.frame_type = STANDARD;//标准帧,
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
