# dArm_control

## 依存関係
1. DYNAMIXEL SDK
    -   Install  
        `$ sudo apt install ros-[ROS Distribution]-dynamixel-sdk`  
        [ROS Distribution]にはROSのディストリビューション(melodic/noetic)を入力
    -   URL : https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/

2. ros_control
    -   Install  
        `$ sudo apt-get install ros-[ROS Distribution]-ros-control`  
        `$ ros-[ROS Distribution]-ros-controllers`
    -   URL : https://wiki.ros.org/ja/ros_control#Install

## 調整
CMakeLists.txtのinclude_directoriesを修正  
`/opt/ros/[ROS Distribution]/include/dynamixel_sdk/`