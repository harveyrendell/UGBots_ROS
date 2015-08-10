#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/tf.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "../Headers/Unit.h"

class Farmer : public Unit
{
private:
	int previous_right_distance;
	int previous_left_distance;
	int previous_front_distance;
	bool currently_turning;

	double rotx;
	double roty;
	double rotz;
	double rotw;
	double angle;
	double desired_angle;
public:
	Farmer(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		this->pose.theta = M_PI/2.0;
		this->pose.px = 5;
		this->pose.py = 10;
		this->speed.linear_x = 20.0;
		this->speed.max_linear_x = 30.0;
		this->speed.angular_z = 0.0;

		this->previous_right_distance = 0;
		this->previous_left_distance = 0;
		this->previous_front_distance = 0;
		this->angle = 0;
		this->desired_angle = M_PI / 2;
		this->currently_turning = false;

		this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
		this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_1/odom",1000, &Farmer::odom_callback, this);
		this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan",1000,&Farmer::laser_callback, this);
	}

	virtual void moveTo(int x, int y){
		
	}

	void odom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		this->pose.px = 5 + msg.pose.pose.position.x;
		this->pose.py = 10 + msg.pose.pose.position.y;
		this->rotx = msg.pose.pose.orientation.x;
    		this->roty = msg.pose.pose.orientation.y;
    		this->rotz = msg.pose.pose.orientation.z;
    		this->rotw = msg.pose.pose.orientation.w;
		this->angle = atan2(2*(roty*rotx+rotw*rotz),rotw*rotw+rotx*rotx-roty*roty-rotz*rotz);
		
		//ROS_INFO("Current y position is: %f", this->pose.py);	
	}

	void laser_callback(sensor_msgs::LaserScan msg)
	{
		//This is the callback function to process laser scan messages
		//you can access the range data from msg.ranges[i]. i = sample number

		if(this->currently_turning)
		{
			ROS_INFO("Current angle is: %f", this->angle);		
			ROS_INFO("Current next desired angle is: %f", this->desired_angle);
			//ROS_INFO("Test laser: TURNING RIGHT %f", msg.ranges[0]);

			//2 clocks
			if((this->angle + 0.31416) >= this->desired_angle)
			{
				this->currently_turning = false;
				this->speed.linear_x = 20.0;
				this->speed.angular_z = 0.0;
				this->desired_angle = this->desired_angle + M_PI / 2.000000;
			}
			return;
		}
		
		
		if(msg.ranges[90] < 3.0)
		{
			this->currently_turning = true;

			this->previous_right_distance = msg.ranges[0];
			this->previous_left_distance = msg.ranges[180];
			this->previous_front_distance = msg.ranges[90];
			
			this->speed.linear_x = 0.0;
			this->speed.angular_z = 5.0;

			//ROS_INFO("Test laser: FRONT %f", msg.ranges[90]);	
		}	
					
	}
};

int main(int argc, char **argv)
{

 //initialize robot parameters
 ros::init(argc, argv, "f1");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

Farmer f1(n);

ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

while (ros::ok())
{
	//messages to stage
	RobotNode_cmdvel.linear.x = f1.speed.linear_x;
	RobotNode_cmdvel.angular.z = f1.speed.angular_z;
        
	//publish the message
	f1.sub_list.node_stage_pub.publish(RobotNode_cmdvel);
	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}

return 0;

} 
