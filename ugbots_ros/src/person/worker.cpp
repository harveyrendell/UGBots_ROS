#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Worker : public Node
{
public:
	Worker(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		this->pose.theta = M_PI/2.0;
		this->pose.px = 5;
		this->pose.py = 10;
		this->speed.linear_x = 20.0;
		this->speed.max_linear_x = 30.0;
		this->speed.angular_z = 0.0;

		this->orientation.previous_right_distance = 0;
		this->orientation.previous_left_distance = 0;
		this->orientation.previous_front_distance = 0;
		this->orientation.angle = 0;
		this->orientation.desired_angle = M_PI / 2;
		this->orientation.currently_turning = false;

		this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
		this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Worker::odom_callback, this);
		this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Worker::laser_callback, this);
	}

	void odom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		this->pose.px = 5 + msg.pose.pose.position.x;
		this->pose.py = 10 + msg.pose.pose.position.y;
		this->orientation.rotx = msg.pose.pose.orientation.x;
    		this->orientation.roty = msg.pose.pose.orientation.y;
    		this->orientation.rotz = msg.pose.pose.orientation.z;
    		this->orientation.rotw = msg.pose.pose.orientation.w;
		this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);

		doAngleCheck();
	}


	void laser_callback(sensor_msgs::LaserScan msg)
	{
		if(this->orientation.currently_turning)
		{
			//2 clocks
			if((this->orientation.angle + (M_PI / (speed.angular_z * 2) ) ) >= this->orientation.desired_angle)
			{
				stopTurn();
			}
			return;
		}
		
		
		if(msg.ranges[90] < 3.0)
		{	
			turnLeft();	
		}	
	}

	void move(){}

	void stop(){
		this->speed.linear_x = 0.0;
		this->speed.angular_z = 0.0;
	}

	void stopTurn(){
		this->orientation.currently_turning = false;
		this->speed.linear_x = 20.0;
		this->speed.angular_z = 0.0;
		this->orientation.desired_angle = this->orientation.desired_angle + M_PI / 2.000000;
	}

	void turnLeft(){
		this->orientation.currently_turning = true;

		this->orientation.previous_right_distance = msg.ranges[0];
		this->orientation.previous_left_distance = msg.ranges[180];
		this->orientation.previous_front_distance = msg.ranges[90];
			
		this->speed.linear_x = 0.0;
		this->speed.angular_z = 5.0;
	}

	void turnRight(){
		this->orientation.currently_turning = true;

		this->orientation.previous_right_distance = msg.ranges[0];
		this->orientation.previous_left_distance = msg.ranges[180];
		this->orientation.previous_front_distance = msg.ranges[90];
			
		this->speed.linear_x = 0.0;
		this->speed.angular_z = -5.0;
	}

	void doAngleCheck(){		
		if(this->orientation.angle < 0)
		{
			this->orientation.angle = this->orientation.angle + 2.000000 * M_PI;
		}

		if(this->orientation.desired_angle > (2.000000 * M_PI))
		{
			this->orientation.desired_angle = M_PI / 2.000000;
		}
		
		if(this->orientation.angle > 2.000000 * M_PI)
		{
			this->orientation.angle = this->orientation.angle - 2.000000 * M_PI;	
		}
	}

	void collisionDetected(){}
};

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
