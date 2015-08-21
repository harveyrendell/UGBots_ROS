#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/visitor.h>

Visitor::Visitor()
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;
}

Visitor::Visitor(ros::NodeHandle &n)
{
	//this->n = n;

	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = -40;
	this->pose.py = -44;
	this->speed.linear_x = 2.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;
	this->state = IDLE;

	this->orientation.previous_right_distance = 0;
	this->orientation.previous_left_distance = 0;
	this->orientation.previous_front_distance = 0;
	this->orientation.angle = 0;
	this->orientation.desired_angle = 0;

	this->orientation.currently_turning = false;
	this->orientation.currently_turning_static = false;

	this->temp_for_testing = false;

	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Visitor::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Visitor::laser_callback, this);
}

void Visitor::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	this->pose.px = -40 + msg.pose.pose.position.x;
	this->pose.py = -44 + msg.pose.pose.position.y;
	this->orientation.rotx = msg.pose.pose.orientation.x;
	this->orientation.roty = msg.pose.pose.orientation.y;
	this->orientation.rotz = msg.pose.pose.orientation.z;
	this->orientation.rotw = msg.pose.pose.orientation.w;
	

	calculateOrientation();

	if(this->temp_for_testing == false)
	{
		this->orientation.desired_angle = this->orientation.angle;
		this->temp_for_testing = true;
	}

	doAngleCheck();		

	checkTurningStatus();

	publish();

	//checkStaticTurningStatus();

	/*ROS_INFO("/position/x/%f",this->pose.px);
	ROS_INFO("/position/y/%f",this->pose.py);
	ROS_INFO("/status/%s/./",enum_to_string(this->state));*/	

	//ROS_INFO("ANGLE: %f",this->orientation.angle);
	//ROS_INFO("DESIRED ANGLE: %f", this->orientation.desired_angle);

}


void Visitor::laser_callback(sensor_msgs::LaserScan msg)
{
	if(msg.ranges[90] < 3.0)
	{
		ROS_INFO("ANGLE: %f",this->orientation.angle);
		ROS_INFO("DESIRED ANGLE: %f", this->orientation.desired_angle);
		turn((M_PI) , 0.0, -5.0);

	}

	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number	
}

void Visitor::checkTurningStatus()
{
		//Implement individually.
		//Change 2-3 to which ever suits your node
		//parse in ur desired linear speed to stopTurn()
		
	if(this->orientation.currently_turning == true)
	{
		if(this->orientation.angle  == this->orientation.desired_angle)
		{
			this->orientation.currently_turning = false;
			this->speed.linear_x = 2.0;
			this->speed.angular_z = 0.0; 	
		}
		return;
	}
		
}

void Visitor::move(){}
void Visitor::stop(){}
void Visitor::turnLeft(){}
void Visitor::turnRight(){}
void Visitor::collisionDetected(){}
char const* Visitor::enum_to_string(State t){ return ""; }

int main(int argc, char **argv)
{	
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "VISITOR");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Visitor node(n);

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
