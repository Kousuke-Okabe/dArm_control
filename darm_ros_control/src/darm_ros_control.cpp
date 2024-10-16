#include <ros/ros.h>
#include <std_msgs/Float64.h>
// #include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <darm.h>
// #include <fstream>
// #include <string>
// #include <iostream>
// #include <sstream>
// #include <stdio.h>
// #include <errno.h>


// static std::mutex JointStates_mutex;
// static sensor_msgs::JointState JointState;
// static std_msgs::Float64 q1,q2,q3, dq1,dq2,dq3, current1,current2,current3;

// void Subscribe_Joint_State(const sensor_msgs::JointState::ConstPtr &JointState){
  // q1.data = JointState->position[0];
  // q2.data = JointState->position[1];
  // q3.data = JointState->position[2];

  // dq1.data = JointState->velocity[0];
  // dq2.data = JointState->velocity[1];
  // dq3.data = JointState->velocity[2];

  // current1.data = JointState->effort[0];
  // current2.data = JointState->effort[1];
  // current3.data = JointState->effort[2];
// }

int main( int argc, char* argv[] ){
  // ROSノード初期化
  // ROS_INFO("main");
  ros::init(argc, argv, "darm_ros_control");
  ros::NodeHandle nh;

  // Publisherの登録
  // ros::Publisher pub_q_pre = nh.advertise<std_msgs::Float64MultiArray>("/angle_present",10);


  // ROS control
  // ROS_INFO("ROS control");
  dArm robot;
  controller_manager::ControllerManager cm(&robot);

  // サンプリング周期設定
  // ROS_INFO("sampling");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Time t = ros::Time::now();
  ros::Rate rate(1000.0); //[Hz]

  while (ros::ok()){
    ros::Duration d = ros::Time::now() - t;
    ros::Time t = ros::Time::now();

    robot.read();
    cm.update(t, d);
    robot.write();

    // ROS_INFO("test data publish");
    // temp.data = 10.0;
    // pub_test.publish(temp);

    rate.sleep();
  }

  return 0;
}
