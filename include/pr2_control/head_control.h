#ifndef HEAD_CONTROL_H
#define HEAD_CONTROL_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class PR2HeadControl
{
private:
  PointHeadClient* point_head_client_;

public:
  PR2HeadControl()
  {
      point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
      while(!point_head_client_->waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the point_head_action server to come up");

  }
  ~PR2HeadControl()
  {
      delete point_head_client_;
  }
  bool look_at(geometry_msgs::PointStamped point)
  {
      bool succeeded = true;
      pr2_controllers_msgs::PointHeadGoal goal;      

      goal.target = point;
      //(pointing_axis defaults to X-axis)
      goal.pointing_frame = "high_def_frame";
      goal.pointing_axis.x = 1;
      goal.pointing_axis.y = 0;
      goal.pointing_axis.z = 0;
      //take at least 0.5 seconds to get there     
      goal.min_duration = ros::Duration(0.5);
      //and go no faster than 1 rad/s
      goal.max_velocity = 1.0;
      //send the goal
      point_head_client_->sendGoal(goal);
      point_head_client_->waitForResult(ros::Duration(3));
      return succeeded;
  }  
};
#endif

