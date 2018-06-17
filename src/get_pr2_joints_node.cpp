#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

using namespace std;

void joint_state_callback(const sensor_msgs::JointState& msg)
{
  /*17 r_upper_arm_roll_joint
    18 r_shoulder_pan_joint
    19 r_shoulder_lift_joint
    20 r_forearm_roll_joint
    21 r_elbow_flex_joint
    22 r_wrist_flex_joint
    23 r_wrist_roll_joint
    31 l_upper_arm_roll_joint
    32 l_shoulder_pan_joint
    33 l_shoulder_lift_joint
    34 l_forearm_roll_joint
    35 l_elbow_flex_joint
    36 l_wrist_flex_joint
    37 l_wrist_roll_joint */
    std::cout << std::fixed;
    std::cout << std::setprecision(2);
    cout <<"right arm: " << msg.position[18] << " ,"<< msg.position[19] << " ,"<< msg.position[17] << " ,"<< msg.position[21] << " ,"<< msg.position[20] << " ,"<< msg.position[22] << " ,"<< msg.position[23]  << endl;
    cout <<"left  arm: " << msg.position[32] << " ,"<< msg.position[33] << " ,"<< msg.position[31] << " ,"<< msg.position[35] << " ,"<< msg.position[34] << " ,"<< msg.position[36] << " ,"<< msg.position[37]  << endl;
    cout <<"torso    : " << msg.position[12] << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_joint_values_node");
    ROS_INFO("The get_joint_values node is started");

    ros::NodeHandle joint_node;
    ros::Subscriber joint_subscriber = joint_node.subscribe("/joint_states", 1, joint_state_callback);

    ros::spin();
    return 0;
}
