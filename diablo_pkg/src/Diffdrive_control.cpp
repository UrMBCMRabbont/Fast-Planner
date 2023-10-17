#include <Eigen/Geometry>
#include <diablo_pkg/MotionCmd.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include "std_msgs/Empty.h"
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <cmath>
#include <ros/ros.h>


class DiffdriveNodelet : public nodelet::Nodelet
{
public:
    DiffdriveNodelet(){}
    void onInit(void);

private:
  void publishDiabloCmd(void);
  void position_cmd_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void imu_callback(const sensor_msgs::Imu& imu);
  diablo_pkg::MotionCmd PIDcontrol(void);

  ros::Publisher motion_cmd_pub;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber imu_sub_;
 
  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  Eigen::Vector3d cur_pos_, cur_vel_, cur_acc_;
  double          des_yaw_, des_yaw_dot_;
  double          current_yaw_;
  double          kp, kR_, kOm_;
};


void DiffdriveNodelet::position_cmd_callback(
  const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;

  publishDiabloCmd();
}

void DiffdriveNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    cur_pos_ = Eigen::Vector3d(odom->pose.pose.position.x,
                                odom->pose.pose.position.y,
                                odom->pose.pose.position.z);
    cur_vel_ = Eigen::Vector3d(odom->twist.twist.linear.x,
                                odom->twist.twist.linear.y,
                                odom->twist.twist.linear.z);

    current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

}

diablo_pkg::MotionCmd
DiffdriveNodelet::PIDcontrol(void)
{   
    kp = 1.0;
    float errorx = des_pos_[0] - cur_pos_[0];
    float errory = des_pos_[1] - cur_pos_[1];
    // TODO: match IMU with map coordinate 
    float err_theta = current_yaw_ - std::atan2(errory, errorx);
    float err_dist = std::sqrt( std::pow(errorx,2) + std::pow(errory,2) );
    
    diablo_pkg::MotionCmd velocity_cmd_;
    velocity_cmd_.linear_vel = kp*err_dist;
    velocity_cmd_.angular_vel = kp*err_theta;

    return velocity_cmd_;
}

void DiffdriveNodelet::publishDiabloCmd()
{   
    diablo_pkg::MotionCmd cmd_msg = PIDcontrol();
    motion_cmd_pub.publish(cmd_msg); 
}


void
DiffdriveNodelet::onInit(void)
{
    ros::NodeHandle nh(getPrivateNodeHandle());

    // publish to robot
    motion_cmd_pub = nh.advertise<diablo_pkg::MotionCmd>("/diablo_motion_cmd", 10);
    // receive target pos 
    position_cmd_sub_ = nh.subscribe("/position_cmd", 10, &DiffdriveNodelet::position_cmd_callback, this,
                            ros::TransportHints().tcpNoDelay());
    // receive current pos (simulation)
    odom_sub_ = nh.subscribe("odom", 10, &DiffdriveNodelet::odom_callback, this,
                            ros::TransportHints().tcpNoDelay());




}