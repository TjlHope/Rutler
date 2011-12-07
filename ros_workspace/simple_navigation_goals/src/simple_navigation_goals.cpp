#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include "std_msgs/String.h"
using std::string;

double x = 0;
double y = 0;
double z = 0;
double p = 0;
double q = 0;
double r = 0;
double w = 1;
string room = "310";

void converterCallback(const std_msgs::String::ConstPtr& msg)
{
	
	ROS_INFO("hello %s",msg->data);

 	if (msg->data.c_str() == room) {
		ROS_INFO("WOOOOOO");
		x = 99.5;
		y = 99.5;
		w = 1.0;
 	}
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  ros::Subscriber sub = n.subscribe("chatter", 1000, converterCallback);
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  ROS_INFO("Sending goal");
  ROS_INFO("%.2f",x);
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  ros::spin();

  return 0;
}

