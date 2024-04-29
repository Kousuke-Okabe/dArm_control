#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

// Kinematics
#define ARM_LENGTH_LINK1                0.09
#define ARM_LENGTH_LINK2                0.09
#define ARM_LENGTH_LINK3                0.053

// #define Sigma       0.01

double Sigma = 0.0;

Eigen::Vector3d Tau_present_torque;
Eigen::Vector3d q_present_angle;
Eigen::MatrixXd jacobi(2,3), Jp(3,2);
Eigen::Matrix2d JJt;
Eigen::Vector3d Us;
Eigen::MatrixXd E =  Eigen::MatrixXd::Identity(3,3);


// Function of subscribe /dArm/joint_states
void Subscribe_Joint_State(const sensor_msgs::JointState::ConstPtr &JointState){
  // ROS_INFO("subscribe joint_state");
  // ROS_INFO("Sigma:%f", Sigma);

  q_present_angle(0) = (double)JointState->position[0];
  q_present_angle(1) = (double)JointState->position[1];
  q_present_angle(2) = (double)JointState->position[2];
  
  Tau_present_torque(0) = (double)JointState->effort[0];
  Tau_present_torque(1) = (double)JointState->effort[1];
  Tau_present_torque(2) = (double)JointState->effort[2];

  // // Jacobi matrix
  // jacobi(0,0) = - ARM_LENGTH_LINK1*sin(q_present_angle(0)) - ARM_LENGTH_LINK2*sin(q_present_angle(0)+q_present_angle(1)) - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
  // jacobi(0,1) = - ARM_LENGTH_LINK2*sin(q_present_angle(0)+q_present_angle(1)) - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
  // jacobi(0,2) = - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
  // jacobi(1,0) = ARM_LENGTH_LINK1*cos(q_present_angle(0)) + ARM_LENGTH_LINK2*cos(q_present_angle(0)+q_present_angle(1)) + ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
  // jacobi(1,1) = ARM_LENGTH_LINK2*cos(q_present_angle(0)+q_present_angle(1)) + ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
  // jacobi(1,2) = ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
  
  // // Collisition detection
  // JJt = jacobi * jacobi.transpose();
  // Jp = jacobi.transpose() * JJt.inverse();

  // Us = (E - Jp * jacobi) * Tau_present_torque;

  // if(Us.norm() > Sigma)   ROS_INFO("||Us||=%02.3f>%02.3f,  Collision", (double)Us.norm(),Sigma);
  // else                    ROS_INFO("||Us||=%02.3f<=%02.3f, without Collision", (double)Us.norm(),Sigma);
}

int main(int argc, char* argv[]){
  ROS_INFO("Collision detection");

  // Initialize ROS node
  ros::init(argc, argv, "collision_detection");
  ros::NodeHandle nh;

  // Registrate Subscribers
  ros::Subscriber sub_joint_state = nh.subscribe("/joint_states",10, Subscribe_Joint_State);

  // Load Parameter
  Sigma = nh.param<double>("Threshold_of_CollisionDetection",0.1);

  // Calcurate frequency
  ros::Rate rate(2.0);

  // Initalise variables
  Tau_present_torque << 0.0, 0.0, 0.0;

  while(ros::ok()){
    ros::spinOnce();

    // Jacobi matrix
    jacobi(0,0) = - ARM_LENGTH_LINK1*sin(q_present_angle(0)) - ARM_LENGTH_LINK2*sin(q_present_angle(0)+q_present_angle(1)) - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(0,1) = - ARM_LENGTH_LINK2*sin(q_present_angle(0)+q_present_angle(1)) - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(0,2) = - ARM_LENGTH_LINK3*sin(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(1,0) = ARM_LENGTH_LINK1*cos(q_present_angle(0)) + ARM_LENGTH_LINK2*cos(q_present_angle(0)+q_present_angle(1)) + ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(1,1) = ARM_LENGTH_LINK2*cos(q_present_angle(0)+q_present_angle(1)) + ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    jacobi(1,2) = ARM_LENGTH_LINK3*cos(q_present_angle(0)+q_present_angle(1)+q_present_angle(2));
    
    // Collisition detection
    JJt = jacobi * jacobi.transpose();
    Jp = jacobi.transpose() * JJt.inverse();

    Us = (E - Jp * jacobi) * Tau_present_torque;

    if(Us.norm() > Sigma)   ROS_INFO("||Us||=%02.3f>%02.3f,  Collision", (double)Us.norm(),Sigma);
    else                    ROS_INFO("||Us||=%02.3f<=%02.3f, without Collision", (double)Us.norm(),Sigma);

    rate.sleep();
  }

  return 0;
}