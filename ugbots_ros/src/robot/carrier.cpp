#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/carrier.h>

Carrier::Carrier()
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;

	moving = false;
	undergoing_task = false;
	swag = false;

	tempx = -10.0;
	tempy = -48.0;
	temprad = 0.0;
}

Carrier::Carrier(ros::NodeHandle &n)
{
	//this->n = n;

	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;

	moving = false;
	undergoing_task = false;
	swag = false;

	tempx = -10.0;
	tempy = -48.0;
	temprad = 0.0;

	sent = false;
	std::string ns = n.getNamespace();
	ns.erase(ns.begin());
	robotDetails.ns = ns;

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Carrier::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Carrier::laser_callback, this);
	sub_ground = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,&Carrier::ground_callback, this);
	bin_sub = n.subscribe<ugbots_ros::Position>("bin",1000,&Carrier::bin_loc_callback, this);
	carrier_alert = n.subscribe<ugbots_ros::bin_status>("/alert",1000,&Carrier::bin_callback,this);
	carrier_alert_pub = n.advertise<ugbots_ros::bin_status>("/alert",1000);
	core_alert = n.advertise<ugbots_ros::robot_details>("/idle_carriers",1000);
}

void Carrier::bin_callback(ugbots_ros::bin_status msg)
{
	localBinStatus.bin_x = msg.bin_x;
	localBinStatus.bin_y = msg.bin_y;
	localBinStatus.bin_stat = msg.bin_stat;

}

char const* Carrier::enum_to_string(State t){
    switch(t){
        case IDLE:
            return "IDLE";
        case TRAVELLING:
            return "TRAVELLING";
        case CARRYING:
            return "CARRYING";
        case AVOIDING:
            return "AVOIDING";
        case STOPPED:
            return "STOPPED";     
        default:
            return "INVALID ENUM";
    }
 }


void Carrier::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	this->pose.px = -10 + msg.pose.pose.position.x;
	this->pose.py = -48 + msg.pose.pose.position.y;	
	orientation.rotx = msg.pose.pose.orientation.x;
	orientation.roty = msg.pose.pose.orientation.y;
	orientation.rotz = msg.pose.pose.orientation.z;
	orientation.rotw = msg.pose.pose.orientation.w;
	orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),
	orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*
	orientation.roty-orientation.rotz*orientation.rotz);
	//ROS_INFO("/position/x/%f", this->pose.px);
	//ROS_INFO("/position/y/%f", this->pose.py);
	//ROS_INFO("/status/%s/./", enum_to_string(state));


	if(localBinStatus.bin_stat == "FULL")
	{
		move_to(localBinStatus.bin_x,localBinStatus.bin_y);
		/*if(move_to(bin_x,bin_y))
		{
			binStatus.bin_stat = "EMPTY";
		}*/
	}
}


void Carrier::laser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number	
}

void Carrier::ground_callback(nav_msgs::Odometry msg)
{
	if (state == IDLE && !sent) {
		robotDetails.x = msg.pose.pose.position.x;
		robotDetails.y = msg.pose.pose.position.y;
		core_alert.publish(robotDetails);
		sent = true;
	}
}

void Carrier::bin_loc_callback(ugbots_ros::Position pos)
{
	ROS_INFO("Robot given coordinates x: %f, y: %f", pos.x, pos.y);
}

void Carrier::turn(bool clockwise, double desired_angle, double temprad) {
	double current_angular_z;

	//desired angle of turn added to robots current angle facing
	orientation.desired_angle = desired_angle + temprad;

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
	if (orientation.desired_angle-3*(current_angular_z/10) >= orientation.angle) {
		orientation.currently_turning = true;
	//if desired angle is reached, robot stops turning and moves again 
	} else {
		speed.angular_z = 0.0;
		orientation.currently_turning = false;
		//stopped = false;
		state = STOPPED;
		zero_angle = orientation.desired_angle;
	}
}
void Carrier::moveX(double distance, double px) {
	double x = distance + px;
	double distance_x = x - pose.px;
	if (distance_x < 0.20001) {
		state = STOPPED;
		speed.linear_x = 0.0;
	}
}

void Carrier::moveY(double distance, double py) {
	double y = distance + py;
	double distance_y = y - pose.py;
	ROS_INFO("y:%f",distance);
	if (distance_y < 0.20001) {
		state = IDLE;
		speed.linear_x = 0.0;
	}
}

bool Carrier::move_to(double x, double y)
{
	/*if(abs(pose.px - x) < 0.00001 && abs(pose.py - y) < 0.00001)
	{
		speed.linear_x = 0.0;
		return true;
	}
	else
	{*/
		state = TRAVELLING;
		speed.linear_x = 1.0;
		moveX(abs(x - tempx), tempx);
		if (speed.linear_x == 0.0) 
		{
			turn(false, M_PI/2, 0.0);
			if (speed.angular_z == 0.0)
			{
				state = TRAVELLING;
				speed.linear_x = 1.0;
				moveY(abs(y - tempy),tempy);
				temprad = orientation.angle;
			}	
		}
	//}
	return false;
}


void Carrier::move_forward(double distance)
{	
	
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

	if(distance_z < 2.000001){
		speed.linear_x = 0.0;
		moving = false;
	}
}

void Carrier::move(){}
void Carrier::stop(){}
void Carrier::turnLeft(){}
void Carrier::turnRight(){}
void Carrier::collisionDetected(){}

int main(int argc, char **argv)
{	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "CARRIER");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Carrier node(n);

node.binStatus.bin_stat = "EMPTY";

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

while (ros::ok())
{
	node.publish();	

	node.carrier_alert_pub.publish(node.binStatus);

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
