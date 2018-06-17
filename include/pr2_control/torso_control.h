#ifndef TORSO_CONTROL_H
#define TORSO_CONTROL_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

class PR2TorsoControl
{
private:
  TorsoClient* torso_client_;


public:
  PR2TorsoControl()
  {
      torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);
      while(!torso_client_->waitForServer(ros::Duration(5.0)))
          ROS_INFO("Waiting for the torso action server to come up");

  }
  ~PR2TorsoControl()
  {
      delete torso_client_;
  }
  bool set(pr2_controllers_msgs::SingleJointPositionGoal g)
  {
      bool succeeded = true;

      torso_client_->sendGoal(g);
      torso_client_->waitForResult();
      if(torso_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("The gripper failed!");
        succeeded = false;
      }
      return succeeded;
  }  
  bool set(double height)
  {
      bool succeeded = true;
      if(height < 0 || height > 0.31)
      {
          cout << "invalid height value";
          return false;
      }

      pr2_controllers_msgs::SingleJointPositionGoal goal;
      goal.position = height;
      goal.min_duration = ros::Duration(2.0);
      goal.max_velocity = 0.5;
      torso_client_->sendGoal(goal);
      torso_client_->waitForResult();
      if(torso_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("The gripper failed!");
        succeeded = false;
      }

      return succeeded;
  }
};
#endif

