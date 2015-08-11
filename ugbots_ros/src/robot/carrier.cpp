#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Carrier : public Node
{
public:
	bool moving = false;
	bool undergoing_task = false;
	double tempx;
	double tempy;
	bool swag = false;





	Carrier(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		pose.theta = M_PI/2.0;
		pose.px = 10;
		pose.py = 20;
		speed.linear_x = 2.0;
		speed.max_linear_x = 3.0;
		speed.angular_z = 0.0;

		sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_3/cmd_vel",1000);
		sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_3/odom",1000, &Carrier::odom_callback, this);
		sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_3/base_scan",1000,&Carrier::laser_callback, this);
		//sub_list.sub_comm = n.subscribe<idk>("robot3/what_ever",1000,&Carrier::comm_callback, this);
	}


	void odom_callback(nav_msgs::Odometry msg)
	{
		//This is the call back function to process odometry messages coming from Stage. 	
		this->pose.px = msg.pose.pose.position.x;
		this->pose.py = -33 + msg.pose.pose.position.y;
		//ROS_INFO("Current x position is: %f", this->pose.px);
		//ROS_INFO("Current y position is: %f", this->pose.py);
		orientation.rotx = msg.pose.pose.orientation.x;
		orientation.roty = msg.pose.pose.orientation.y;
		orientation.rotz = msg.pose.pose.orientation.z;
		orientation.rotw = msg.pose.pose.orientation.w;


		this->pose.theta = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),
			orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*
			orientation.roty-orientation.rotz*orientation.rotz);
		//ROS_INFO("Current y position is: %f", this->pose.theta);
		test();
	}


	void laser_callback(sensor_msgs::LaserScan msg)
	{
		
		//This is the callback function to process laser scan messages
		//you can access the range data from msg.ranges[i]. i = sample number	
	}

	void comm_callback(){

		//if(msg.status == "ready")
		//{


		//}


	}

	void test(){
		move_forward(10);
		move_forward(30);

	}

    return false;

	bool at_point(double x, double y){
		if()

	}
	void turn(double desiredAngle)
	{
		speed.angular_z = 3.0;
		speed.angular_z = 0.0;

	}

	void move(double x, double y)
	{


	}

	void move_forward(double distance)
	{


		if(!undergoing_task){
			undergoing_task = true;
			speed.linear_x = 2.0;
			if(!moving)
			{
				tempx = pose.px;
				tempy = pose.py;
				moving = true;
			}

			double x = distance * cos(pose.theta) + tempx;
			double y = distance * sin(pose.theta) + tempy;

			double distance_x = x - pose.px;
			double distance_y = y - pose.py;
			double distance_z = sqrt(pow(distance_x,2) + pow(distance_y,2));
		}
	}

	void move(){
		speed.linear_x = 2.0;
	}
	void stop(){
		speed.linear_x = 0.0;
	}
	void turnLeft(){
		speed.angular.z = 1;
	}
	void turnRight(){
		speed.angular.z = -1;
	}
	void collisionDetected(){}
};

int main(int argc, char **argv)
{	
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "CARRIER");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Carrier node(n);

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
