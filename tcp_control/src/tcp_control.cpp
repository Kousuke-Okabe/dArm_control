#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

// Kinematics
#define ARM_LENGTH_LINK1                0.09
#define ARM_LENGTH_LINK2                0.09
#define ARM_LENGTH_LINK3                0.053
#define Kp                              10

using namespace Eigen;

// static std_msgs::Float64 r_ref_x, r_ref_y;
Vector3d q_present_angle;
Vector2d r_command_position;

void Subscribe_Joint_State(const sensor_msgs::JointState::ConstPtr &JointState){
  q_present_angle(0) = (double)JointState->position[0];
  q_present_angle(1) = (double)JointState->position[1];
  q_present_angle(2) = (double)JointState->position[2];
}

void Subscribe_TCP_Commant(const std_msgs::Float64MultiArray &TCP_cmd){
  r_command_position(0) = (double)TCP_cmd.data[0];
  r_command_position(1) = (double)TCP_cmd.data[1];
}

int main(int argc, char* argv[]){
  ROS_INFO("TCP_controller")

  // ROSノード初期化
  ros::init(argc, argv, "tcp_control");
  ros::NodeHandle nh;

  // Publisherの登録
  ros::Publisher pub_q1_cmd = nh.advertise<std_msgs::Float64>("/dArm/velocity_controller_q1/command",10);
  ros::Publisher pub_q2_cmd = nh.advertise<std_msgs::Float64>("/dArm/velocity_controller_q2/command",10);
  ros::Publisher pub_q3_cmd = nh.advertise<std_msgs::Float64>("/dArm/velocity_controller_q3/command",10);

  // Subscriberの登録
  ros::Subscriber sub_joint_state = nh.subscribe("/dArm/joint_states",10, Subscribe_Joint_State);
  ros::Subscriber sub_tcp_commant = nh.subscribe("/dArm/tcp_commant",10, Subscribe_TCP_Commant);

  // 周期設定
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Time t = ros::Time::now();
  ros::Rate rate(100.0); //[Hz]

  Vector2d r_present_position;
  Vector3d q_command_velocity;
  MatrixXd jacobi(2,3);

  static std_msgs::Float64 dq1,dq2,dq3;

  q_present_angle << 0.0, 0.0, 0.0;
  r_command_position << 0.0, ARM_LENGTH_LINK1+ARM_LENGTH_LINK2+ARM_LENGTH_LINK3;

  while(ros::ok()){
    ros::Duration d = ros::Time::now() - t;
    ros::Time t = ros::Time::now();

    // Kinematics
    r_present_position(0) = ARM_LENGTH_LINK1*cos(q_present_angle(0)) + ARM_LENGTH_LINK2*cos(q_present_angle(0)+q_present_angle(1)) + ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    r_present_position(1) = ARM_LENGTH_LINK1*sin(q_present_angle(0)) + ARM_LENGTH_LINK2*sin(q_present_angle(0)+q_present_angle(1)) + ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));

    // Jacobi matrix
    jacobi(0,0) = - ARM_LENGTH_LINK1*sin(q_present_angle(0)) - ARM_LENGTH_LINK2*sin(q_present_angle(0)+q_present_angle(1)) - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(0,1) = - ARM_LENGTH_LINK2*sin(q_present_angle(0)+q_present_angle(1)) - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(0,2) = - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(1,0) = ARM_LENGTH_LINK1*cos(q_present_angle(0)) + ARM_LENGTH_LINK2*cos(q_present_angle(0)+q_present_angle(1)) + ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(1,1) = ARM_LENGTH_LINK2*cos(q_present_angle(0)+q_present_angle(1)) + ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(1,2) = ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    
    // TCP position controller
    q_command_velocity = jacobi.transpose() * Kp * (r_command_position - r_present_position);

    // Publisht the q_command_velocity
    dq1.data = q_command_velocity(0);
    dq2.data = q_command_velocity(1);
    dq3.data = q_command_velocity(2);

    pub_q1_cmd.publish(dq1);
    pub_q2_cmd.publish(dq2);
    pub_q3_cmd.publish(dq3);

    rate.sleep();
  }

  return 0;
}