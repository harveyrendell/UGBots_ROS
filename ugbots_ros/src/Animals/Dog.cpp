#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include "Robot.h"
#include "Unit.h"

class Dog : public Animal
{
public:
	Dog(ros::NodeHandle &n);
};

Dog::Dog(ros::NodeHandle &n): Animal(&n)

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
		RobotNode_cmdvel.linear.x = cb.speed.linear_x;
		RobotNode_cmdvel.angular.z = cb.speed.angular_z;
	        
		//publish the message
		cb.sub_list.node_stage_pub.publish(RobotNode_cmdvel);
		
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;

}
