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

  std::string controller_name;
  nh_.param<std::string>("controller_name", controller_name, "joint_trajectory_controller");
  nh_.param<std::string>("target_link", target_link_, "link_6");

  traj_client_ = new TrajClient(controller_name + "/follow_joint_trajectory", true);
  traj_pub_ =  nh_.advertise<trajectory_msgs::JointTrajectory>("/" + controller_name + "/command", 10);
    
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
  srv.request.ik_request.group_name = "manipulator";
  srv.request.ik_request.robot_state.joint_state = joint_state_;
  srv.request.ik_request.avoid_collisions = true;
  geometry_msgs::PoseStamped ref_pose;
  
  double dt = 0.01;
  geometry_msgs::Pose act_pose = pose_stamped_.pose;

  ref_pose.header.frame_id = "base_link";
  ref_pose.header.stamp = ros::Time::now();
  ref_pose.pose.position.x = act_pose.position.x + msg->twist.linear.x * dt;
  ref_pose.pose.position.y = act_pose.position.y + msg->twist.linear.y * dt;
  ref_pose.pose.position.z = act_pose.position.z + msg->twist.linear.z * dt;
#if 1
  ref_pose.pose.orientation.x = act_pose.orientation.x;
  ref_pose.pose.orientation.y = act_pose.orientation.y;
  ref_pose.pose.orientation.z = act_pose.orientation.z;
  ref_pose.pose.orientation.w = act_pose.orientation.w;
#else
  ref_pose.pose.orientation.x = 0;
  ref_pose.pose.orientation.y = 0;
  ref_pose.pose.orientation.z = 0;
  ref_pose.pose.orientation.w = 1;
#endif
  
  ROS_INFO_STREAM("ref_pose" << ref_pose);

  srv.request.ik_request.pose_stamped = ref_pose;

  if (ik_client_.call(srv))
  {
    // ROS_INFO_STREAM("response: " << srv.response);
  }
  else
  {
    ROS_ERROR("Failed to call service /compute_ik");
  }
  if (srv.response.error_code.val < 0)
  {
    ROS_ERROR("IK error");
    return;
  }
  ref_joint_state_ = srv.response.solution.joint_state;

  if (0)
  {
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
  else
  {
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
    traj.header.frame_id = "base_link";
    traj.joint_names = ref_joint_state_.name;
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = srv.response.solution.joint_state.position;
    point.velocities.resize(ref_joint_state_.name.size());
    point.accelerations.resize(ref_joint_state_.name.size());
    point.time_from_start = ros::Duration(1.0);
    traj.points.push_back(point);

    traj_pub_.publish(traj);
  }
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
  srv.request.fk_link_names.clear();
  srv.request.fk_link_names.push_back(target_link_);
  srv.request.robot_state.joint_state = joint_state_;
  if (fk_client_.call(srv))
  {
    if(srv.response.error_code.val < 0)
    {
      ROS_ERROR("****FK error %d", srv.response.error_code.val);
      return;
    }
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

