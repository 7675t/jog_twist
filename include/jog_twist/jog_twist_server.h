#ifndef JOG_ARM_SERVER_H
#define JOG_ARM_SERVER_H

#include <Eigen/Eigenvalues>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <tf/transform_listener.h>

namespace jog_arm {
 
/**
 * Class JogArmServer - Provides the jog_arm action.
 */
class JogArmServer
{
public:
  /**
   * @breif: Default constructor for JogArmServer Class.
   */
  JogArmServer(std::string move_group_name, std::string cmd_topic_name);
  
protected:
  
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  
  void commandCB(geometry_msgs::TwistStampedConstPtr msg);

  void jointStateCB(sensor_msgs::JointStateConstPtr msg);

  float overallScaling;
  Vector6d scaleCommand(const geometry_msgs::TwistStamped& command, const Vector6d& scalar) const;
  
  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &J) const;
  
  bool addJointIncrements(sensor_msgs::JointState &output, const Eigen::VectorXd &increments) const;
  
  bool checkConditionNumber(const Eigen::MatrixXd &matrix, double threshold) const;

  ros::NodeHandle nh_;
  
  ros::Subscriber joint_sub_, cmd_sub_;
  
  moveit::planning_interface::MoveGroupInterface arm_;
  const robot_state::JointModelGroup* joint_model_group_;
  robot_state::RobotStatePtr kinematic_state_;
  
  sensor_msgs::JointState current_joints_;
  
  std::vector<std::string> joint_names_;
  
  ros::AsyncSpinner spinner_; // Motion planner requires an asynchronous spinner
  
  tf::TransformListener listener_;
};

} // namespace jog_arm

#endif // JOG_ARM_SERVER_H
