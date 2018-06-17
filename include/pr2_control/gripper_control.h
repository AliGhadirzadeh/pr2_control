#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

#define OPEN_GRIPPER true
#define CLOSE_GRIPPER false

using namespace std;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class PR2GripperControl
{
private:
  GripperClient* l_gripper_client_;
  GripperClient* r_gripper_client_;

public:
  PR2GripperControl()
  {
      l_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
      r_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
      while(!r_gripper_client_->waitForServer(ros::Duration(5.0)))
          ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
      while(!l_gripper_client_->waitForServer(ros::Duration(5.0)))
          ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
  }
  ~PR2GripperControl()
  {
      delete l_gripper_client_;
      delete r_gripper_client_;
  }
  bool set(std::string gripper, pr2_controllers_msgs::Pr2GripperCommandGoal cmd)
  {
      bool succeeded = true;
      if (gripper == "right_gripper")
      {
          r_gripper_client_->sendGoal(cmd);
          r_gripper_client_->waitForResult(ros::Duration(4.0));
          if(r_gripper_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("The gripper failed!");
            succeeded = false;
          }
      }
      else if (gripper == "left_gripper")
      {
          l_gripper_client_->sendGoal(cmd);
          l_gripper_client_->waitForResult(ros::Duration(4.0));
          if(l_gripper_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("The gripper failed!");
            succeeded = false;
          }
      }
      return succeeded;
  }  
};
#endif

