# jog_twist

## Subscribed topic

- /joint_states (sensor_msgs/JointState)

  Current joint states (angle, velocity, acceleration)

- cmd_vel (geoemtry_msgs/TwistStamped)
  
  Commnad velocity of the target frame

## Required service / action

- /compute_ik
  
  Service to get inverse kinematics by MoveIt!

- /compute_fk
  
  Service to get forward kinematics by MoveIt!
  
- /follow_joint_trajecotry (control_msgs/FollowJointTrajectoryAction)
  
  Action to command joint trajectory
  
## 

メインループの処理

- 現在の手先pose = compute_fk(joint_state)
- 目標の手先pose = 現在のpose + cmd_vel * dt
- 目標の関節角度 = comput_ik(目標の手先pose)
- follow_joint_trajectory.setgoal(目標の関節角度)

- IK,FKはそんなに時間かからないはずだけど、ちょっと危険な気がする
- cmd_velのコールバック内じゃなくて、メインループで実装したほうが良い

