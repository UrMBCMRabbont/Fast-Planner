#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>

class DiffdriveControl
{
public:
  DiffdriveControl();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d& position, double, double);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAcc(const Eigen::Vector3d& acc);
  void setExec_state(const int exec_state);

  void calculateControl(const Eigen::Vector3d& des_pos,
                        const Eigen::Vector3d& des_vel,
                        const Eigen::Vector3d& des_acc, const double des_yaw,
                        const double des_yaw_dot, const Eigen::Vector3d& kx,
                        const Eigen::Vector3d& kv);
  const Eigen::Vector3d&    getComputedForce(void);
  const geometry_msgs::Twist& getComputedVel(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  int fsm_exec_state;
  // Inputs for the controller
  double          mass_;
  double          g_;
  double current_yaw_;
  double current_yaw_dot_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;

  // Outputs of the controller
  Eigen::Vector3d    force_;
  geometry_msgs::Twist Cmdvel;

};

