#ifndef JOINT_TRAJECTORY_ACTION_H
#define JOINT_TRAJECTORY_ACTION_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class PR2ArmJointTrajectory
{
private:
  TrajClient* r_traj_client_;
  TrajClient* l_traj_client_;

public:
  PR2ArmJointTrajectory()
  {
      r_traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
      while(!r_traj_client_->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the  r_joint_trajectory_action server");
      l_traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);
      while(!l_traj_client_->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the  l_joint_trajectory_action server");
  }
  ~PR2ArmJointTrajectory()
  {
      delete r_traj_client_;
      delete l_traj_client_;
  }

  bool follow_trajectory(std::string arm, trajectory_msgs::JointTrajectory traj)
  {
      bool succeeded = true;
      pr2_controllers_msgs::JointTrajectoryGoal goal;
      goal.trajectory = traj;

      if (arm == "right_arm")
      {
          r_traj_client_->sendGoal(goal);
          r_traj_client_->waitForResult(ros::Duration(8.0));
          if(r_traj_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
              succeeded = false;
              ROS_WARN("r_traj_client was not successful");
          }
      }
      else if (arm == "left_arm")
      {
          l_traj_client_->sendGoal(goal);
          l_traj_client_->waitForResult(ros::Duration(8.0));
          if(l_traj_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
              succeeded = false;
              ROS_WARN("l_traj_client was not successful");
          }
      }
      return succeeded;
  }

  bool move_arm(std::string arm, vector<trajectory_msgs::JointTrajectoryPoint> joint_target)
  {
      int n_targets = joint_target.size();
      pr2_controllers_msgs::JointTrajectoryGoal goal = prepare_goal(arm, n_targets);
      bool succeeded = true;
      for(int n = 0; n< n_targets; n++)
      {
          for (int j = 0; j < 7; ++j)
          {
              goal.trajectory.points[n].velocities[j] = joint_target[n].velocities[j];
              goal.trajectory.points[n].positions[j] = joint_target[n].positions[j];
          }
          goal.trajectory.points[n].time_from_start = joint_target[n].time_from_start;
      }
      if (arm == "right_arm")
      {
          r_traj_client_->sendGoal(goal);

          r_traj_client_->waitForResult();
          if(r_traj_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
              succeeded = false;
              ROS_WARN("r_traj_client was not successful");
          }
      }
      else if (arm == "left_arm")
      {
          l_traj_client_->sendGoal(goal);
          l_traj_client_->waitForResult(ros::Duration(8.0));
          if(l_traj_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
              succeeded = false;
              ROS_WARN("l_traj_client was not successful");
          }
      }
      return succeeded;
  }
  pr2_controllers_msgs::JointTrajectoryGoal prepare_goal(std::string arm, int n_targets)
  {
      pr2_controllers_msgs::JointTrajectoryGoal goal;

      if(arm== "right_arm")
          arm = "r_";
      else
          arm = "l_";

      goal.trajectory.points.resize(n_targets);
      goal.trajectory.joint_names.push_back(arm+"shoulder_pan_joint");
      goal.trajectory.joint_names.push_back(arm+"shoulder_lift_joint");
      goal.trajectory.joint_names.push_back(arm+"upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back(arm+"elbow_flex_joint");
      goal.trajectory.joint_names.push_back(arm+"forearm_roll_joint");
      goal.trajectory.joint_names.push_back(arm+"wrist_flex_joint");
      goal.trajectory.joint_names.push_back(arm+"wrist_roll_joint");
      for(int n = 0; n < n_targets; n++)
      {
          goal.trajectory.points[n].velocities.resize(7);
          goal.trajectory.points[n].positions.resize(7);
      }

      return goal;
  }
};

#endif

