#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <iostream>
#include <stdlib.h>
#include "../Headers/Unit.h"

class Farmer : public Unit
{
public:
	Farmer(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		this->pose.theta = M_PI/2.0;
		this->pose.px = 5;
		this->pose.py = 10;
		this->speed.linear_x = 2.0;
		this->speed.max_linear_x = 3.0;
		this->speed.angular_z = 0.0;

		this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
		this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_1/odom",1000, &Farmer::odom_callback, this);
		this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan",1000,&Farmer::laser_callback, this);
	}

	virtual void moveTo(int x, int y){
		
	}

	void odom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		this->pose.px = 5 + msg.pose.pose.position.x;
		this->pose.py = 10 + msg.pose.pose.position.y;
		//ROS_INFO("Current x position is: %f", this->pose.px);
		//ROS_INFO("Current y position is: %f", this->pose.py);	
	}

	void laser_callback(sensor_msgs::LaserScan msg)
	{
		//This is the callback function to process laser scan messages
		//you can access the range data from msg.ranges[i]. i = sample number

		if(msg.ranges[90] < 3.0)
		{
			this->speed.linear_x = 0.0;
			this->speed.angular_z = 40.0;
			ROS_INFO("Test laser: %f", msg.ranges[90]);	
		}
		else
		{
			this->speed.linear_x = 2.0;
			this->speed.angular_z = 0.0;
		}				
	}
};

int main(int argc, char **argv)
{

 //initialize robot parameters
 ros::init(argc, argv, "f1");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

Farmer f1(n);

ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

while (ros::ok())
{
	//messages to stage
	RobotNode_cmdvel.linear.x = f1.speed.linear_x;
	RobotNode_cmdvel.angular.z = f1.speed.angular_z;
        
	//publish the message
	f1.sub_list.node_stage_pub.publish(RobotNode_cmdvel);
	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}

return 0;

} 
