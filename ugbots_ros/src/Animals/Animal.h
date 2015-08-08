#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include "Robot.h"
#include "Unit.h"

class Animal : public Unit
{
public:
	Animal(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		this->pose.theta = M_PI/2.0;
		this->pose.px = 10;
		this->pose.py = 20;
		this->speed.linear_x = 0.0;
		this->speed.max_linear_x = 3.0;
		this->speed.angular_z = 20.0;

		this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
		this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Template::odom_callback, this);
		this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Template::laser_callback, this);

	}

	virtual void moveTo(int x, int y){
		
	}

	void odom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		this->pose.px = 5 + msg.pose.pose.position.x;
		this->pose.py = 10 + msg.pose.pose.position.y;
		ROS_INFO("Current x position is: %f", this->pose.px);
		ROS_INFO("Current y position is: %f", this->pose.py);
	}


	void laser_callback(sensor_msgs::LaserScan msg)
	{
		//This is the callback function to process laser scan messages
		//you can access the range data from msg.ranges[i]. i = sample number
		
	}
};

