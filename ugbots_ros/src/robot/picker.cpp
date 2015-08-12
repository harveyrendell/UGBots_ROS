#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/picker.h>

Picker::Picker()
{
	init();
}

Picker::Picker(ros::NodeHandle &n)
{
	//setting base attribute defaults
	init();

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Picker::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Picker::laser_callback, this);
}

void Picker::init()
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 3.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 2.0;
	state = IDLE;
}

void Picker::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	pose.px = 5 + msg.pose.pose.position.x;
	pose.py = 10 + msg.pose.pose.position.y;
	ROS_INFO("Current x position is: %f", pose.px);
	ROS_INFO("Current y position is: %f", pose.py);
}


void Picker::laser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
}

void Picker::move(){}
void Picker::stop(){}
void Picker::turnLeft(){}
void Picker::turnRight(){}
void Picker::collisionDetected(){}

int main(int argc, char **argv)
{

//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "PICKER");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Picker node(n);

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
