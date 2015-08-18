#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/worker.h>

Worker::Worker()
{
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
	this->checkedThisRot = false;
}

Worker::Worker(ros::NodeHandle &n)
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
	this->checkedThisRot = false;

	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Worker::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Worker::laser_callback, this);
}

void Worker::odom_callback(nav_msgs::Odometry msg)
{
	//gets the current position and angle and sets it to the object's fields 	
	this->pose.px = -40 + msg.pose.pose.position.x;
	this->pose.py = -44 + msg.pose.pose.position.y;
	this->orientation.rotx = msg.pose.pose.orientation.x;
		this->orientation.roty = msg.pose.pose.orientation.y;
		this->orientation.rotz = msg.pose.pose.orientation.z;
		this->orientation.rotw = msg.pose.pose.orientation.w;
	
	calculateOrientation();

	doAngleCheck();		

	checkTurningStatus();

	checkStaticTurningStatus();

	if(state == IDLE)
	{
		state = PATROLLING;
	}
	else
	{
		if(orientation.currently_turning_static == true)
		{
			state = SAWDOG;
		}
		else
		{
			state = PATROLLING;
		}
	}
	

	ROS_INFO("/position/x/%f",this->pose.px);
	ROS_INFO("/position/y/%f",this->pose.py);
	ROS_INFO("/status/%s/./",enum_to_string(this->state));		
	
}


void Worker::laser_callback(sensor_msgs::LaserScan msg)
{		
	if(msg.ranges[90] < 5.5 && this->orientation.currently_turning == false && this->orientation.currently_turning_static == false) // stop when 5 meteres from the wall is reached directly to the front
	{				
		this->orientation.previous_right_distance = msg.ranges[0];
		this->orientation.previous_left_distance = msg.ranges[180];
		this->orientation.previous_front_distance = msg.ranges[90];
		//turnRight();	//turn left
		turnLeft();
	}	

	if(this->orientation.currently_turning == false && this->orientation.currently_turning_static == false)
	{
		if(this->checkedThisRot == false)
		{
			for(int i=100; i<130; i++)
			{
				if(msg.ranges[i] < 4.5)
				{
					spinOnTheSpot();
				}
			}
		}
	}
}

void Worker::move(){}

//Stops the node
void Worker::stop()
{
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 0.0;
}

//Stops the node turning. Linear velocity will be set to default (1.0)
//Update the next desired angle
void Worker::stopTurn()
{
	this->orientation.currently_turning = false;
	this->speed.linear_x = 2.0;
	this->speed.angular_z = 0.0;
	
	//ROS_INFO("Stop Turn", "");	
}

void Worker::stopTurnStatic()
{
	this->orientation.currently_turning_static = false;
	this->speed.linear_x = 2.0;
	this->speed.angular_z = 0.0;	

	
	//ROS_INFO("Stop Turn Static", "");	
}

//Turn left
void Worker::turnLeft()
{
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI / 2.000000);
	this->speed.linear_x = 0.5;
	this->speed.angular_z = 5.0;

	if(orientation.desired_angle != M_PI)
	{
		checkedThisRot = false;
	}

	//ROS_INFO("Turn Left Desired Angle: %f", this->orientation.desired_angle);	
}

//Static turn left
void Worker::spinOnTheSpot()
{
	this->orientation.currently_turning_static = true;
	this->orientation.desired_angle = (M_PI);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 5.0;
	this->checkedThisRot = true;
	
	//ROS_INFO("Spin on the spot", "");	
}

//Turn right
void Worker::turnRight()
{
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle - (M_PI / 2.000000);
	this->speed.linear_x = 0.5;
	this->speed.angular_z = -5.0;

	
	//ROS_INFO("Turn Right", "");	
}

//Angle translation for easier interpretation
void Worker::doAngleCheck()
{		
	//if -ve rads, change to +ve rads
	if(this->orientation.angle < 0)
	{
		this->orientation.angle = this->orientation.angle + 2.000000 * M_PI;
	}
	//if the desired angle is > 2pi, changed the desired angle to pi/2 
	if(this->orientation.desired_angle > (2.000000 * M_PI))
	{
		this->orientation.desired_angle = M_PI / 2.000000;
	}
	//if the current angle is 2pi or more, translate the angle to 0< x <2pi 
	if(this->orientation.angle > 2.000000 * M_PI)
	{
		this->orientation.angle = this->orientation.angle - 2.000000 * M_PI;	
	}
}

//
void Worker::checkTurningStatus()
{
	if(this->orientation.currently_turning == true)
	{
		//ROS_INFO("LEVEL 1", "");
		if((this->orientation.angle + (M_PI / (speed.angular_z * 2) ) ) >= this->orientation.desired_angle)
		{
			//ROS_INFO("LEVEL 2", "");
			stopTurn(); // stop the turn when desired angle is reacahed (2 clocks before the estimated angle)
		}
		return;
	}
}

//
void Worker::checkStaticTurningStatus()
{
	if(this->orientation.currently_turning_static == true)
	{		
		if((this->orientation.angle + (M_PI / (speed.angular_z * 2) ) ) >= this->orientation.desired_angle)
		{
			if((this->orientation.angle + (M_PI / (speed.angular_z * 2) ) ) <= this->orientation.desired_angle + 0.05)
			{	
				stopTurnStatic();	
			}
		}
		return;
	}
}

//calculates current orientation using atan2
void Worker::calculateOrientation()
{	
	this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);
}
	
void Worker::collisionDetected(){}

char* Worker::enum_to_string(State t)
{
	switch(t){
		case IDLE:
			return "IDLE";
		case PATROLLING:
			return "PATROLLING";
		case RESPONDING:
			return "RESPONDING";
		case SAWDOG:
			return "SAWDOG";
		default:
			return "";
	}
}

int main(int argc, char **argv)
{	
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "WORKER");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Worker node(n);

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
