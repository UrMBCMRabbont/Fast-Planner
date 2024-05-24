#include <iostream>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <simulation_cmd/DiffdriveControl.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/SO3Command.h>


// class DiffdriveCtrlNodelet : public nodelet::Nodelet
// {
// public:
//   DiffdriveCtrlNodelet()
//     : position_cmd_updated_(false)
//     , position_cmd_init_(false)
//     , des_yaw_(0)
//     , des_yaw_dot_(0)
//     , current_yaw_(0)
//   {
//   }

//   void onInit(void);

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// private:
//   void publishCmdVel(void);
//   void position_cmd_callback(
//     const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
//   void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);

//   DiffdriveControl      controller_;
//   ros::Publisher  so3_command_pub_;
//   ros::Publisher  cmd_vel_pub_;
//   ros::Subscriber odom_sub_;
//   ros::Subscriber position_cmd_sub_;

//   bool        position_cmd_updated_, position_cmd_init_;
//   std::string frame_id_;

//   Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
//   double          des_yaw_, des_yaw_dot_;
//   double          current_yaw_;
//   bool            use_external_yaw_;
// };



void publishCmdVel(void);
void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);

DiffdriveControl controller_;
ros::Publisher  so3_command_pub_;
ros::Publisher  cmd_vel_pub_;
ros::Subscriber odom_sub_;
ros::Subscriber position_cmd_sub_;
ros::Subscriber kino_fsm_sub_;

int         fsm_exec_state;
bool        position_cmd_updated_, position_cmd_init_;
std::string frame_id_;

Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
double          des_yaw_, des_yaw_dot_;
double          current_yaw_, current_yaw_dot_;
bool            use_external_yaw_;

void fsm_state_callback(std_msgs::Int32 exec_state_){ 
  int exec_state = int(exec_state_.data);
  controller_.setExec_state(exec_state);
}

void publishCmdVel(void)
{
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                               des_yaw_dot_, kx_, kv_);
  const Eigen::Vector3d& force = controller_.getComputedForce();
  const geometry_msgs::Twist& twist_command = controller_.getComputedVel();

  so3_command_pub_.publish(twist_command);
}

void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  publishCmdVel();
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
  current_yaw_dot_ = odom->twist.twist.angular.z;

  controller_.setPosition(position, current_yaw_, current_yaw_dot_);
  controller_.setVelocity(velocity);

  if (position_cmd_init_)
  {
    if (!position_cmd_updated_)
      publishCmdVel();
    position_cmd_updated_ = false;
  }
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "Diffdrive_control_nodelet");
  ros::NodeHandle node("~");

  double mass;
  node.param("mass", mass, 0.5);
  controller_.setMass(mass);

  so3_command_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 25);
  odom_sub_ = node.subscribe("odom", 10, &odom_callback, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = node.subscribe("position_cmd", 10, &position_cmd_callback, ros::TransportHints().tcpNoDelay());
  kino_fsm_sub_ = node.subscribe<std_msgs::Int32>("/kino_fsm_state", 10, fsm_state_callback);


  ros::spin();

  return 0;
}