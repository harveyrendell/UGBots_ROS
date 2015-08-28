#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include <node_defs/tractor.h>

Tractor::Tractor()
{
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.max_linear_x = 0.0;
	speed.angular_z = 0.0;
	state = IDLE;
}

Tractor::Tractor(ros::NodeHandle &n)
{
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.max_linear_x = 0.0;
	speed.angular_z = 0.0;
	state = IDLE;

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Tractor::ground_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Tractor::laser_callback, this);

	geometry_msgs::Point point;
	point.x = -36.0; 
	point.y = -12.0;

	action_queue.push(point);
	
	point.x = -36.0; 
	point.y = -28.0;

	action_queue.push(point);

	state = IDLE;

}

void Tractor::ground_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	pose.px = msg.pose.pose.position.x;
	pose.py = msg.pose.pose.position.y;
	orientation.rotx = msg.pose.pose.orientation.x;
	orientation.roty = msg.pose.pose.orientation.y;
	orientation.rotz = msg.pose.pose.orientation.z;
	orientation.rotw = msg.pose.pose.orientation.w;

	calculateOrientation();

	if(action_queue.size() == 1)
	{
		geometry_msgs::Point point;
		if(action_queue.front().y == -12)
		{
			point.x = -36.0;
			point.y = -28.0;
		}
		else
		{
			point.x = -36.0;
			point.y = -12.0;
		}

		action_queue.push(point);
	}

	begin_action_shortest_path(2.0);

	state = TRAVELLING;

	doAngleCheck();		

	checkTurningStatus();

	publish();

	ROS_INFO("/position/x/%f",this->pose.px);
	ROS_INFO("/position/y/%f",this->pose.py);
	ROS_INFO("/status/%s/./", enum_to_string(state));
}


void Tractor::laser_callback(sensor_msgs::LaserScan msg)
{

}

void Tractor::move(){}

void Tractor::stop(){
	speed.linear_x = 0.0;
	speed.angular_z = 0.0;
}
void Tractor::collisionDetected(){}

char const* Tractor::enum_to_string(State t)
{
	switch (t){
		case IDLE:
			return "IDLE";
		case TRAVELLING:
			return "TRAVELLING";
		default:
			return ""; 
	}
}

int main(int argc, char **argv)
{

//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "TRACTOR");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating Tractor instance
Tractor node(n);

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

	while (ros::ok())
	{
		ros::spinOnce();

		loop_rate.sleep();

		++count;
	}

return 0;

}