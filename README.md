# dArm_control

## 依存関係
1. ros_control
    -   Install  
        `$ sudo apt-get install ros-[ROS Distribution]-ros-control`  
        `$ ros-[ROS Distribution]-ros-controllers`
    -   URL : https://wiki.ros.org/ja/ros_control#Install

2. DYNAMIXEL SDK
    -   Install  
        `$ sudo apt install ros-[ROS Distribution]-dynamixel-sdk`  
        [ROS Distribution]にはROSのディストリビューション(melodic/noetic)を入力
    -   URL : https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/

3. U2D2 driver
    -    Install  
          `$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/dynamixel-workbench/master/99-dynamixel-workbench-cdc.rules`  
          `$ sudo cp ./99-dynamixel-workbench-cdc.rules /etc/udev/rules.d/`  
          `$ sudo udevadm control --reload-rules`  
          `$ sudo udevadm trigger`
   -    URL : https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/

## 調整
CMakeLists.txtのinclude_directoriesを修正  
`/opt/ros/[ROS Distribution]/include/dynamixel_sdk/`
