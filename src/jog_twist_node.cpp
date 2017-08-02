#include <jog_twist/jog_twist_node.h>

namespace jog_twist {

JogTwistNode::JogTwistNode(std::string move_group_name)
  : nh_("~")
{
  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &JogTwistNode::joint_state_cb, this);
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &JogTwistNode::cmd_vel_cb, this);
  fk_client_ = nh_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");  
  ik_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");  
  ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

  traj_client_ = new TrajClient("position_trajectory_controller/follow_joint_trajectory", true);

  while(!traj_client_->waitForServer(ros::Duration(60)))
  {
    ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
  }
  ROS_INFO_STREAM("Action server is ok!");  
}

/**
 * @brief Callback function for the topic cmd_vel
 *
 */
void JogTwistNode::cmd_vel_cb(geometry_msgs::TwistStampedConstPtr msg)
{
  moveit_msgs::GetPositionIK srv;
  std::vector<std::string> link_name;
  link_name.push_back("link_j6");
  srv.request.ik_request.group_name = "manipulator";
  srv.request.ik_request.robot_state.joint_state = joint_state_;
  srv.request.ik_request.avoid_collisions = false;
  geometry_msgs::PoseStamped ref_pose;
  
  double dt = 0.1;
  geometry_msgs::Pose act_pose = pose_stamped_.pose;

  ref_pose.header.frame_id = "base_link";
  ref_pose.header.stamp = ros::Time::now();
  ref_pose.pose.position.x = act_pose.position.x + msg->twist.linear.x * dt;
  ref_pose.pose.position.y = act_pose.position.y + msg->twist.linear.y * dt;
  ref_pose.pose.position.z = act_pose.position.z + msg->twist.linear.z * dt;
  ref_pose.pose.orientation.w = 1;

  ROS_INFO_STREAM("ref_pose" << ref_pose);

  srv.request.ik_request.pose_stamped = ref_pose;

  if (ik_client_.call(srv))
  {
    ROS_INFO_STREAM(srv.response.solution);
  }
  else
  {
    ROS_ERROR("Failed to call service /computte_ik");
  }
  ref_joint_state_ = srv.response.solution.joint_state;
  
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.01);
  goal.trajectory.header.frame_id = "base_link";
  goal.trajectory.joint_names = ref_joint_state_.name;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = srv.response.solution.joint_state.position;
  point.velocities.resize(ref_joint_state_.name.size());
  point.accelerations.resize(ref_joint_state_.name.size());
  point.time_from_start = ros::Duration(1.0);
  goal.trajectory.points.push_back(point);

  traj_client_->sendGoal(goal);
}

/**
 * @brief Callback function for the topic joint_state
 *
 */
void JogTwistNode::joint_state_cb(sensor_msgs::JointStateConstPtr msg)
{
  // Check that the msg contains joints
  if (msg->name.empty())
  {
    ROS_ERROR("JogTwsitNode::joint_state_cb - joint_state is empty.");
    return;
  }
  joint_state_ = *msg;

  moveit_msgs::GetPositionFK srv;
  std::vector<std::string> link_name;
  link_name.push_back("link_j6");
  srv.request.fk_link_names = link_name;
  srv.request.robot_state.joint_state = joint_state_;
  if (fk_client_.call(srv))
  {
    pose_stamped_ = srv.response.pose_stamped[0];
    for (int i=0; i<srv.response.fk_link_names.size(); i++)
    {
      // ROS_INFO_STREAM(srv.response.fk_link_names[i] << ":" << srv.response.pose_stamped[i]);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service /computte_fk");
  }    
}

} // namespace jog_arm

/**
 * @brief Main function of the node
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "jog_twist_node");
  jog_twist::JogTwistNode node("manipulator");

  ros::Rate loop_rate(100);
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

