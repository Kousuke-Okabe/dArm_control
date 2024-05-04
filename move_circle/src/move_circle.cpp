#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <bits/stdc++.h>

int main(int argc, char*argv[]){
    ROS_INFO("move_circle");

    // ROSノード初期化
    ros::init(argc, argv, "mov_circle");
    ros::NodeHandle nh;

    // Publisherの登録
    ros::Publisher pub_r_cmd = nh.advertise<std_msgs::Float64MultiArray>("/tcp_command",10);
    
    // Registering Parameter
    double CIRCLE_RADIUS_X = nh.param<double>("CIRCLE_RADIUS_X", 0.02);
    double CIRCLE_RADIUS_Y = nh.param<double>("CIRCLE_RADIUS_Y", 0.02);
    double CIRCLE_POSITION_X = nh.param<double>("CIRCLE_POSITION_X", 0.15);
    double CIRCLE_POSITION_Y = nh.param<double>("CIRCLE_POSITION_Y", 0.0);
    double CIRCLE_FREQUENCY = nh.param<double>("CIRCLE_FREQUENCY", 1);
    int Fs = nh.param<int>("COMMAND_FREQUENCY", 50);
    double ETA_COMMAND = nh.param<double>("ETA_COMMAND", M_PI/2);

    // 周期設定
    double Ts = (double)1/Fs;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Time t0 = ros::Time::now();
    // ros::Time t = t0;
    ros::Rate rate(Fs); //[Hz]

    // Valiable
    // Vector2d r_cmd << 0.15, 0.0;
    std_msgs::Float64MultiArray r_cmd;
    r_cmd.data.resize(3);
    r_cmd.data[0] = CIRCLE_POSITION_X;
    r_cmd.data[1] = CIRCLE_POSITION_Y;
    r_cmd.data[2] = ETA_COMMAND;

    while(ros::ok()){
        ros::Duration t = ros::Time::now() - t0;
        pub_r_cmd.publish(r_cmd);

        r_cmd.data[0] = CIRCLE_RADIUS_X * cos(2*M_PI*CIRCLE_FREQUENCY * t.toSec()) + CIRCLE_POSITION_X;
        r_cmd.data[1] = CIRCLE_RADIUS_Y * sin(2*M_PI*CIRCLE_FREQUENCY * t.toSec()) + CIRCLE_POSITION_Y;

        // ROS_INFO("t : %u.%.09u", t.sec,t.nsec);
        // ROS_INFO("d : %f", t.toSec());

        rate.sleep();
    }
    
    return 0;
}