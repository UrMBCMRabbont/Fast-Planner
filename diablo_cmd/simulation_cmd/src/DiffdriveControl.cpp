#include <iostream>
#include <cmath>
#include <simulation_cmd/DiffdriveControl.h>



#include <ros/ros.h>

DiffdriveControl::DiffdriveControl()
  : mass_(0.5)
  , g_(9.81)
{
  acc_.setZero();
}

void 
DiffdriveControl::setExec_state(const int exec_state)
{
  fsm_exec_state = exec_state;
  // std::cout << "fsm_callback function: " << exec_state << std::endl;
  // std::cout << "setExec_state function: " << fsm_exec_state << std::endl;

}

void
DiffdriveControl::setMass(const double mass)
{
  mass_ = mass;
}

void
DiffdriveControl::setGravity(const double g)
{
  g_ = g;
}

void
DiffdriveControl::setPosition(const Eigen::Vector3d& position, double current_yaw, double current_yaw_dot)
{
  pos_ = position;
  current_yaw_ = current_yaw;
  current_yaw_dot_ = current_yaw_dot;
}

void
DiffdriveControl::setVelocity(const Eigen::Vector3d& velocity)
{
  vel_ = velocity;
}

void
DiffdriveControl::calculateControl(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const double des_yaw, const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv)
{

  force_.noalias() =
    kx.asDiagonal() * (des_pos - pos_) + kv.asDiagonal() * (des_vel - vel_);

  double errx = (des_pos - pos_)[0];
  double erry = (des_pos - pos_)[1];
  double driving_ang = std::atan2(erry, errx);
  double err_ang = (des_yaw - current_yaw_);

  Cmdvel.linear.y = 0;
  Cmdvel.linear.z = 0;
  Cmdvel.angular.x = 0;
  Cmdvel.angular.y = 0;
  
  double linearP_ = std::hypot(errx, erry)*9;
  double linearD_ = std::hypot(des_vel[0], des_vel[1])*1.25;
  Cmdvel.linear.x = linearP_ - linearD_;
  // Cmdvel.angular.z = (driving_ang - current_yaw_);
  Cmdvel.angular.z = err_ang*0.5;
  // Laibon need to find the correct facing angle value 

  if(fsm_exec_state == 1 || (des_acc[0] == 0.0 && des_acc[1] == 0.0)){
    Cmdvel.linear.x = 0.0;
    Cmdvel.angular.z = 0.0;
  }

  ROS_INFO("linear vel %lf", Cmdvel.linear.x);
  // ROS_INFO("Angular vel %lf", Cmdvel.angular.z); 
  ROS_INFO("current_yaw:  %lf", current_yaw_);  
  // ROS_INFO("Driving angle %lf", driving_ang);
  ROS_INFO("des yaw %lf", des_yaw);
  ROS_INFO("Angular error %lf", err_ang);
  // ROS_INFO("linear P gain %lf", linearP_);
  // ROS_INFO("linear D gain %lf", linearD_);


  //  ROS_INFO("Error %lf %lf %lf", (des_pos - pos_).norm(),
  //           (des_vel - vel_).norm(), (des_acc - acc_).norm());
}

const Eigen::Vector3d&
DiffdriveControl::getComputedForce(void)
{
  return force_;
}

const geometry_msgs::Twist&
DiffdriveControl::getComputedVel(void)
{
  return Cmdvel;
}

void
DiffdriveControl::setAcc(const Eigen::Vector3d& acc)
{
  acc_ = acc;
}
