#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Picker : public Node
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
		orientation.desired_angle = desired_angle + zero_angle;

		/*double current_angle = orientation.angle;

		if (current_angle < 0) {
			current_angle = -current_angle;
		}*/

		if (clockwise) {
			speed.angular_z = -M_PI/2;
		} else {
			if (orientation.angle < 0) {
				orientation.angle = M_PI + (M_PI + orientation.angle);
			}
			speed.angular_z = M_PI/2;
		}

		if (orientation.desired_angle-(M_PI/10) > orientation.angle) {
		} else {
			turningLeft = false;
			turningRight = false;
			stopped = false;
			speed.angular_z = 0.0;
			zero_angle = orientation.desired_angle;
			ROS_INFO("Zero angle is: %f", zero_angle);
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
			turningLeft = true;
			stopped = true;
		}

		logic();
		ROS_INFO("Editted angle is: %f", orientation.angle);
	}

	void move(){}
	void stop(){}
	void turnLeft(){}
	void turnRight(){}
	void collisionDetected(){}
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
