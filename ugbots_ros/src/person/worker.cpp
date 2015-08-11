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
		this->speed.linear_x = 1.0;
		this->speed.max_linear_x = 30.0;
		this->speed.angular_z = 0.0;

		this->orientation.previous_right_distance = 0;
		this->orientation.previous_left_distance = 0;
		this->orientation.previous_front_distance = 0;
		this->orientation.angle = 0;
		this->orientation.desired_angle = 0;

		this->orientation.currently_turning = false;

		this->orientation.currently_turning_static = false;

		this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
		this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Worker::odom_callback, this);
		this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Worker::laser_callback, this);
	}

	void odom_callback(nav_msgs::Odometry msg)
	{
		//gets the current position and angle and sets it to the object's fields 	
		this->pose.px = 5 + msg.pose.pose.position.x;
		this->pose.py = 10 + msg.pose.pose.position.y;
		this->orientation.rotx = msg.pose.pose.orientation.x;
    		this->orientation.roty = msg.pose.pose.orientation.y;
    		this->orientation.rotz = msg.pose.pose.orientation.z;
    		this->orientation.rotw = msg.pose.pose.orientation.w;
		
		calculateOrientation();

		doAngleCheck();

	}


	void laser_callback(sensor_msgs::LaserScan msg)
	{
		if(this->orientation.currently_turning)
		{
			
			ROS_INFO("Current Angle: %f", this->orientation.angle + (M_PI / (speed.angular_z * 2) ));	
			ROS_INFO("Desired Angle: %f", this->orientation.desired_angle);	

			if((this->orientation.angle + (M_PI / (speed.angular_z * 2) ) ) >= this->orientation.desired_angle)
			{
				stopTurn(); // stop the turn when desired angle is reacahed (2 clocks before the estimated angle)
			}
			return;
		}
		
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
		
		if(msg.ranges[90] < 18.0) // stop when 5 meteres from the wall is reached directly to the front
		{				
			this->orientation.previous_right_distance = msg.ranges[0];
			this->orientation.previous_left_distance = msg.ranges[180];
			this->orientation.previous_front_distance = msg.ranges[90];
			//turnRight();	//turn left
			turnLeft();
		}	

		if(this->orientation.currently_turning == false && this->orientation.currently_turning_static == false)
		{
			for(int i=100; i<130; i++)
			{
				if(msg.ranges[i] < 10)
				{
					spinOnTheSpot();
				}
			}
		}
	}

	void move(){}

	//Stops the node
	void stop(){
		this->speed.linear_x = 0.0;
		this->speed.angular_z = 0.0;
	}
	
	//Stops the node turning. Linear velocity will be set to default (1.0)
	//Update the next desired angle
	void stopTurn(){
		this->orientation.currently_turning = false;
		this->speed.linear_x = 1.0;
		this->speed.angular_z = 0.0;
	}

	void stopTurnStatic()
	{
		this->orientation.currently_turning_static = false;
		this->speed.linear_x = 1.0;
		this->speed.angular_z = 0.0;	
	}

	//Turn left
	void turnLeft(){
		this->orientation.currently_turning = true;
		this->orientation.desired_angle = this->orientation.desired_angle + (M_PI / 2.000000);
		this->speed.linear_x = 0.5;
		this->speed.angular_z = 5.0;
	}

	//Static turn left
	void spinOnTheSpot(){
		this->orientation.currently_turning_static = true;
		this->orientation.desired_angle = (M_PI);
		this->speed.linear_x = 0.0;
		this->speed.angular_z = 5.0;
	}

	//Turn right
	void turnRight(){
		this->orientation.currently_turning = true;
		this->orientation.desired_angle = this->orientation.desired_angle - (M_PI / 2.000000);
		this->speed.linear_x = 0.5;
		this->speed.angular_z = -5.0;
	}

	//Angle translation for easier interpretation
	void doAngleCheck(){		
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
	
	//calculates current orientation using atan2
	void calculateOrientation()
	{	
		this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);
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
