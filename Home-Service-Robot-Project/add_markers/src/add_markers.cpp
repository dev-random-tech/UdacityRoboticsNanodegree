#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <complex>


float pickUp[3] = {3.0, 5.0, 1.0};
float dropOff[3] = {-1.0, 0.0, 1.0};
float thresh[2] = {0.3, 0.01};


bool atPickUp = false;
bool atDropOff = false;
bool pickUpDone = false;
bool dropOffDone = false;


void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  
if (std::abs(pickUp[0] -msg->pose.pose.position.x) < thresh[0] && std::abs(pickUp[1] -msg->pose.pose.position.y) < thresh[0] && std::abs(pickUp[2] -msg->pose.pose.orientation.w) < thresh[1])
   { 
    if(!atPickUp)
    {
     atPickUp = true;
    }
   }else{atPickUp = false;}

if (std::abs(dropOff[0] -msg->pose.pose.position.x) < thresh[0] && std::abs(dropOff[1] -msg->pose.pose.position.y) < thresh[0] && std::abs(dropOff[2] -msg->pose.pose.orientation.w) < thresh[1])
  { 
    if(!atDropOff)
    {
     atDropOff = true;
    }
   }else{atDropOff = false;}

}

int main( int argc, char** argv )
{
  ROS_INFO("Main");
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, chatterCallback);

  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pickUp[0];
    marker.pose.position.y = pickUp[1];
    marker.pose.position.z = 0;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = pickUp[2];

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
   marker_pub.publish(marker);
   ROS_INFO("Pick-up marker displayed");
   
   while(!atPickUp)
   {
    ros::spinOnce();
   }
   
   if(atPickUp && !pickUpDone)
   {
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ROS_INFO("Pick-up marker removed");
    pickUpDone = true;
   }  
   
   while(!atDropOff)
   {
    ros::spinOnce();
   }

   if(atDropOff && !dropOffDone)
   {
    marker.pose.position.x = dropOff[0];
    marker.pose.position.y = dropOff[1];
    marker.pose.orientation.w = dropOff[2];;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    ROS_INFO("Drop-off marker displayed");
    dropOffDone = true;
    ros::Duration(10.0).sleep();
   }  
    return 0;
  }
 
}
