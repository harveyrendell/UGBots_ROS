#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/picker.h>

Picker::Picker(ros::NodeHandle &n)
{
private:
	Orientation orientation;
	bool turningLeft = false;
	bool turningRight = false;
	bool stopped = false;
	double zero_angle = 0.0;
public:
	Picker(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		pose.theta = M_PI/2.0;
		pose.px = 10;
		pose.py = 20;
		speed.linear_x = 1.0;
		speed.max_linear_x = 3.0;
		speed.angular_z = 0.0;

		sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
		sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Picker::odom_callback, this);
		sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Picker::laser_callback, this);
	}

	void logic() {
		if (turningLeft) {
			turn(false, M_PI/2);
		} else if (turningRight) {
			turn(true, M_PI/2);
		}

		if (stopped) {
			speed.linear_x = 0.0;
		} else {
			speed.linear_x = 1.0;
		}
	}

	void turn(bool clockwise, double desired_angle) {
		double current_angular_z;

		//desired angle of turn added to robots current angle facing
		orientation.desired_angle = desired_angle + zero_angle;

		//deduct one rotation if desired angle exceed full rotation
		if (orientation.desired_angle > 2*M_PI) {
			orientation.desired_angle = orientation.desired_angle - 2*M_PI;
		}

		//for when turn is set to be clockwise
		if (clockwise) {
			if (orientation.angle > 0) {
				orientation.angle = -2*M_PI + orientation.angle;
			}
			speed.angular_z = -M_PI/2;
			current_angular_z = -speed.angular_z;
			orientation.angle = -orientation.angle;
		} else {
			if (orientation.angle < 0) {
				orientation.angle = 2*M_PI + orientation.angle;
			}
			speed.angular_z = M_PI/2;
			current_angular_z = speed.angular_z;
		}

		//turn until desired angle is reached, taking into account of the 2 clock time ahead
		if (orientation.desired_angle-2*(current_angular_z/10) >= orientation.angle) {
		//if desired angle is reached, robot stops turning and moves again 
		} else {
			turningLeft = false;
			turningRight = false;
			stopped = false;
			speed.angular_z = 0.0;
			zero_angle = orientation.desired_angle;
		}
	}

	virtual void moveTo(int x, int y){
		
	}

	void odom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		pose.px = 5 + msg.pose.pose.position.x;
		pose.py = 10 + msg.pose.pose.position.y;
		ROS_INFO("Current x position is: %f", pose.px);
		ROS_INFO("Current y position is: %f", pose.py);
		orientation.rotx = msg.pose.pose.orientation.x;
		orientation.roty = msg.pose.pose.orientation.y;
		orientation.rotz = msg.pose.pose.orientation.z;
		orientation.rotw = msg.pose.pose.orientation.w;
		orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),
			orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*
			orientation.rotz);
		ROS_INFO("Current angle is: %f", orientation.angle);
	}


	void laser_callback(sensor_msgs::LaserScan msg)
	{
		//This is the callback function to process laser scan messages
		//you can access the range data from msg.ranges[i]. i = sample number
		if (msg.ranges[90] < 3) {
			turningRight = true;
			stopped = true;
		}

		logic();
	}

	void Picker::move(){}
	void Picker::stop(){}
	void Picker::turnLeft(){}
	void Picker::turnRight(){}
	void Picker::collisionDetected(){}
};

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
