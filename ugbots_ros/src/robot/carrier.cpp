#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/carrier.h>

//Constructor with no nodehandler argument for testing
Carrier::Carrier()
{
	init();
}

Carrier::Carrier(ros::NodeHandle &n)
{
	init();

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Carrier::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Carrier::laser_callback, this);
}

void Carrier::init()
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 30.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 20.0;
	state = IDLE;
}

void Carrier::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	this->pose.px = 5 + msg.pose.pose.position.x;
	this->pose.py = 10 + msg.pose.pose.position.y;
	ROS_INFO("Current x position is: %f", this->pose.px);
	ROS_INFO("Current y position is: %f", this->pose.py);
}

void Carrier::laser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number	
}

void Carrier::move(){}
void Carrier::stop(){}
void Carrier::turnLeft(){}
void Carrier::turnRight(){}
void Carrier::collisionDetected(){}

int main(int argc, char **argv)
{	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "CARRIER");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Carrier node(n);

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

while (ros::ok())
{
	node.publish();
	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
