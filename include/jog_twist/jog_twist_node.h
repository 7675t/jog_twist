#ifndef JOG_TWIST_NODE_H
#define JOG_TWIST_NODE_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;
 
namespace jog_twist {

/**
 * Class JogTwistNode - Provides the jog_twist action.
 */
class JogTwistNode
{
public:
  /**
   * @breif: Default constructor for JogTwistNode Class.
   */
  JogTwistNode(std::string move_group_name);
  
protected:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_, cmd_vel_sub_;
  ros::ServiceClient fk_client_, ik_client_;

  TrajClient* traj_client_;

  sensor_msgs::JointState joint_state_;
  geometry_msgs::PoseStamped pose_stamped_;
  sensor_msgs::JointState ref_joint_state_;
  
  void cmd_vel_cb(geometry_msgs::TwistStampedConstPtr msg);
  void joint_state_cb(sensor_msgs::JointStateConstPtr msg);
};

} // namespace jog_twist

#endif // JOG_TWIST_NODE_H
