#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

float pickUp[3] = {3.0, 5.0, 1.0};
float dropOff[3] = {-1.0, 0.0, 1.0};


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pickUp[0];
  goal.target_pose.pose.position.y = pickUp[1];
  goal.target_pose.pose.orientation.w = pickUp[2] ;

  ROS_INFO("Sending Pick up goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     
    {
     ROS_INFO("Hooray, Robot reached PICK-UP......");
     ros::Duration(5.0).sleep();
     goal.target_pose.pose.position.x = dropOff[0];
     goal.target_pose.pose.position.y = dropOff[1];
     goal.target_pose.pose.orientation.w = dropOff[2];
     ROS_INFO("Sending goal sending drop off goal");
     ac.sendGoal(goal);
     ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     {ROS_INFO("Hooray, Robot reached DROP OFF......");
     ros::Duration(5.0).sleep();}
  else
     {ROS_INFO("Robot failed to reach Drop off location for some reason");}
    }
  else
    {ROS_INFO("Robot failed to reach pick up location for some reason");}

  return 0;
}
