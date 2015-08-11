#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <iostream>
#include <stdlib.h>
#include "../Headers/Unit.h"

class Dog : public Unit
{
public:
	bool endOfPath = false;

	Dog(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		this->pose.theta = M_PI/2.0;
		this->pose.px = 10;
		this->pose.py = 20;
		this->speed.linear_x = 2.0;
		//this->speed.max_linear_x = 3.0;
		this->speed.angular_z = 0.0;

		this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
		this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Dog::odom_callback, this);
		this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000, &Dog::laser_callback, this);
	}

	virtual void moveTo(int x, int y){
		
	}

	void odom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		this->pose.px = -16 + msg.pose.pose.position.x;
		this->pose.py = 42.5 + msg.pose.pose.position.y;


		//ROS_INFO("Current x position is: %f", this->pose.px);
		//ROS_INFO("Current y position is: %f", this->pose.py);

		if (msg.pose.pose.position.x >= 32) {
			this->speed.linear_x = 0.0;
			this->speed.angular_z = 0.0;
			endOfPath = true;
			//ROS_INFO("Lets check the angle x: %f", msg.pose.pose.orientation.x);
			//ROS_INFO("Lets check the angle y: %f", msg.pose.pose.orientation.y);
			ROS_INFO("Lets check the angle z: %f", msg.pose.pose.orientation.z);
			ROS_INFO("Lets check the angle w: %f", msg.pose.pose.orientation.w);
			if (msg.pose.pose.orientation.z =! 1){
				this->speed.angular_z = 3.0;
			}

			if((this->orientation.angle + (M_PI / (speed.angular_z * 2))) >= this->orientation.desired_angle)
				{
				stopTurn(); // stop the turn when desired angle is reacahed (2 clocks before the estimated angle)
				}
		}
	}


	void laser_callback(sensor_msgs::LaserScan msg)
	{
		//This is the callback function to process laser scan messages
		//you can access the range data from msg.ranges[i]. i = sample number
		bool detection = false;
		for(int a = 0 ; a < 180; a++){
			if (msg.ranges[a] < 7) {
				detection = true;
				continue;
			} 
		}

		if (detection == true){
			this->speed.linear_x = 0.0;
			ROS_INFO("BARK BARK BARK BARK!");
		} else {
			if (endOfPath == false){
				this->speed.linear_x = 2.0;
			}
		}

		
	}

	void calculateOrientation()
	{	
		this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);
	}
};

int main(int argc, char **argv)
{	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "DOG");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

Dog dg(n);

/*//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
//ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 
//use the one below when using launch, use the one above when testing individual robot
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000); 

//subscribe to listen to messages coming from stage
//robo.setSubs(n);
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &R0::StageOdom_callback, &robo);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&R0::StageLaser_callback, &robo);*/


ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

while (ros::ok())
{
	//messages to stage
	RobotNode_cmdvel.linear.x = dg.speed.linear_x;
	RobotNode_cmdvel.angular.z = dg.speed.angular_z;
        
	//publish the message
	dg.sub_list.node_stage_pub.publish(RobotNode_cmdvel);
	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
