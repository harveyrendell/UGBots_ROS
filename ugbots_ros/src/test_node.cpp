#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <unit.h>

class Tester : public Unit
{
public:
	Tester(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		pose.theta = M_PI/2.0;
		pose.px = 10;
		pose.py = 20;
		speed.linear_x = 0.0;
		speed.max_linear_x = 3.0;
		speed.angular_z = 0.0;

		sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
		sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Tester::odom_callback, this);
		sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Tester::laser_callback, this);
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

	void move()
	{
		speed.linear_x = speed.max_linear_x;
	}

	void stop()
	{
		speed.linear_x = 0.0;
		speed.angular_z = 0.0;
	}
};

int main(int argc, char **argv)
{

ros::init(argc, argv, "test");
ros::NodeHandle n;
Tester bot(n);
ros::Rate loop_rate(10);
int count = 0;

while (ros::ok())
{
	if (count == 0)
	{
		bot.move();
	}

	if (count > 100)
	{
		bot.speed.angular_z = 3.0;
	}

	if (count > 200)
	{
		bot.stop();
	}

	bot.publish();	
	ros::spinOnce();
	loop_rate.sleep();
	count++;
}
return 0;

}