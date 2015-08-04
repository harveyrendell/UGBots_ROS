#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include "Robot.h"

class R0 : public Robot
{
public:
	R0();

	void moveTo(int x, int y)
	{
		ROS_INFO("Current x position is: %f", px);
		ROS_INFO("Current y position is: %f", py);

	}

	int stop(int t)
	{
		return 0;
	}

	void StageOdom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		px = 5 + msg.pose.pose.position.x;
		py = 10 + msg.pose.pose.position.y;
		ROS_INFO("Current x position is: %f", px);
		ROS_INFO("Current y position is: %f", py);
	}


	void StageLaser_callback(sensor_msgs::LaserScan msg)
	{
		//This is the callback function to process laser scan messages
		//you can access the range data from msg.ranges[i]. i = sample number
		
	}

	void setSubs(ros::NodeHandle& n)
	{
		ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &R0::StageOdom_callback, this);
		ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&R0::StageLaser_callback, this);
	}
};

R0::R0(void)
{
	theta = M_PI/2.0;
	px = 10;
	py = 20;
	
	//Initial velocity
	linear_x = 100.0;
	angular_z = 20.0;
}

int main(int argc, char **argv)
{

 //initialize robot parameters
	R0 robo;
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "RobotNode0");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

//subscribe to listen to messages coming from stage
robo.setSubs(n);

ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

while (ros::ok())
{
	robo.moveTo(1,1);

	//messages to stage
	RobotNode_cmdvel.linear.x = robo.linear_x;
	RobotNode_cmdvel.angular.z = robo.angular_z;
        
	//publish the message
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}

return 0;

}
