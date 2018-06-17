#include <ros/ros.h>
#include "pr2_control/arm_trajectory_control.h"
#include "pr2_control/gripper_control.h"


int main(int argc, char** argv)
{
  // Init the ROS node
  std::cout << "The demo program started" <<std::endl;
  ros::init(argc, argv, "demo_node");

  PR2ArmJointTrajectory arm_controller;

  trajectory_msgs::JointTrajectoryPoint p;
  vector<trajectory_msgs::JointTrajectoryPoint> p_array;
  p.positions.resize(7);
  p.velocities.resize(7);
  p.time_from_start = ros::Duration(5.0);

  for(int i = 0; i < 7; i++)
  {
      p.positions[i] = 0;
      p.velocities[i] = 0;
  }
  p_array.push_back(p);
  arm_controller.move_arm("right_arm",p_array);
  sleep(2.0);
  arm_controller.move_arm("left_arm",p_array);
  sleep(2.0);


  PR2GripperControl gripper_controller;
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0.08;
  gripper_goal.command.max_effort = 35;
  gripper_controller.set("left_gripper", gripper_goal);
  gripper_controller.set("right_gripper", gripper_goal);
  sleep(1.0);
  gripper_goal.command.position = 0.01;
  gripper_controller.set("left_gripper", gripper_goal);
  gripper_controller.set("right_gripper", gripper_goal);

  while(ros::ok())
  {

  }
  return 0;
}
