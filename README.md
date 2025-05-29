# ROS Packages for DingLAB USB2CAN 
USB转2路CAN模块购买地址： https://e.tb.cn/h.TBC18sl6EZKXUjL?tk=C5g5eLgyMf6HU071
## 前期准备
本程序需要与肥猫机器人公司USB2CAN模块配合使用，请准备好模块与模块说明书、模块SDK，`并按照说明书使用install.sh文件安装USB2CAN规则文件，或手动安装规则文件，安装方法：`
1. 进入项目目录下的SDK文件夹
```bash
cd USB2CAN_ROS/SDK
```
2. 复制规则文件usb_can.rules 到/etc/udev/rules.d/
```bash
sudo cp usb_can.rules /etc/udev/rules.d/
```
3. 运行下面的命令，使udev规则生效
```bash
sudo udevadm trigger
```


```USB转2路CAN模块购买地址：```

https://e.tb.cn/h.TBC18sl6EZKXUjL?tk=C5g5eLgyMf6HU071

```说明书以及SDK下载地址：```
https://pan.baidu.com/s/1EwYDNQ0jMKyTSvJEEcj6aw?pwd=10ob

## 安装
(接下来的操作均假定在以下用户目录下进行: `~/catkin_ws/src`)

```
cd ~/catkin_ws/src
git clone https://github.com/SOULDE-Studio/USB2CAN_ROS.git
cd ..
catkin_make
```

## 启动ROS节点
* source环境变量
    ```
    source devel/setup.bash
    ```
* 启动模块驱动节点
    ```
    roslaunch usb2can_ros usb2can.launch 
    ```
    开启这一节点会自动检测到已插入的设备并发布`/{DEVICE_NAME}/can_rx`（如`/USB2CAN0/can_rx`）,话题用于向用户控制程序节点发送USB2CAN模块接收到的can数据，同时订阅`/{DEVICE_NAME}/can_tx`话题用于接收用户控制程序发送的控制数据并发送数据到USB2CAN模块。`/{DEVICE_NAME}`为接入的USB2CAN模块的设备名称：`USB2CAN0`、`USB2CAN1`、`USB2CAN2`、`USB2CAN3`，接入几个模块就会发布几个设备名称的话题，最多4个模块，模块的ID与设备名称一一对应,,且不能重复（ID设置方法见USB2CAN模块说明书https://pan.baidu.com/s/1EwYDNQ0jMKyTSvJEEcj6aw?pwd=10ob）。
* 启动示例收发程序
    ```
    rosrun usb2can_ros user_node _device:=USB2CAN0
    ```
    开启这一节点会自动订阅`/_device/can_rx`话题，同时发布`/_device/can_tx`话题循环发布固定数据，同时打印发送和接收数据的次数，并计算丢包率，`_device`为启动节点时设置的设备名称参数，由用户根据启动模块驱动节点发布的话题名称选择。
    
>> 经测试,使用can通道2向can通道1发送数据频率为8000hz时，丢包率在0.15%左右。
