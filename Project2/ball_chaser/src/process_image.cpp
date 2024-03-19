#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float x_linear, float z_angular)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = x_linear;
    srv.request.angular_z = z_angular;
	
    if (!client.call(srv))
	{
        ROS_ERROR("Failed to call service command_robot");
    }	
}

float correction(int wrong_loc, int correct_centre)
{
	int error = wrong_loc - correct_centre; //In accordance with ROS Right hand rule
	//It will move the whole width length when correction in drive_robot(0.0,1.5)
	//if error is error columns, then rectification = ...
	float rectification = (error*1.5)/(correct_centre*2);
	return rectification;
}

void process_image_callback(const sensor_msgs::Image live_image)
{
    int white_colour = 255;
	int white_count = 0;
	int ball_mid;
	int i_cols = 0;
	int i_col;
	float moving_to;

    for (int i = 0; i < live_image.height*live_image.step; i+=3)
	{
    	i_col = ((i/3)%live_image.step);
	
        if (live_image.data[i] == white_colour && live_image.data[i+1] == white_colour && live_image.data[i+2] == white_colour) 
		{
			white_count++;
			i_cols += i_col;
		}
    }

	if (white_count == 0)
	{
		ball_mid = -1;
		drive_robot(0.0,0.0);
	}
	else
	{
		ball_mid = i_cols/white_count;
    	moving_to = correction(ball_mid,(live_image.step/2));

    	if (moving_to == 0)
    	{
        	drive_robot(5.0,0.0); //No error, move straight
    	}	
		else
		{
			drive_robot(0.0,moving_to); //Put values in so that, ball comes to centre
		}
	}
	

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber img_sub = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ros::spin();

    return 0;
}


