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

	idle_status_sent = false;

	temprad = 0.0;
}

Carrier::Carrier(ros::NodeHandle &n)
{
	//this->n = n;

	//setting base attribute defaults
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;
	orientation.currently_turning = false;

	x_completed = false;
	x_started = false;
	y_completed = false;
	x_started = false;

	idle_status_sent = false;
	std::string ns = n.getNamespace();
	ns.erase(ns.begin());
	robotDetails.ns = ns;

	core_alert = n.advertise<ugbots_ros::robot_details>("/idle_carriers",1000);
	sub_bin = n.subscribe<ugbots_ros::Position>("bin", 1000, &Carrier::bin_loc_callback, this);

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_1/base_pose_ground_truth",1000, &Carrier::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan",1000,&Carrier::laser_callback, this);
	carrier_alert = n.subscribe<ugbots_ros::bin_status>("/alert",1000,&Carrier::bin_callback,this);
	carrier_alert_pub = n.advertise<ugbots_ros::bin_status>("/alert",1000);

	geometry_msgs::Point point;
	point.x = 36.0;
	point.y = -4.0;
	action_queue.push(point);
	point.y = 4.0;
	point.x = 38.0;
	action_queue.push(point);
	point.y = -2.0;
	point.x = -4.0;
	action_queue.push(point);
}

void Carrier::bin_callback(ugbots_ros::bin_status msg)
{
	localBinStatus = msg;
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
	pose.px = msg.pose.pose.position.x;
	pose.py = msg.pose.pose.position.y;
	orientation.rotx = msg.pose.pose.orientation.x;
	orientation.roty = msg.pose.pose.orientation.y;
	orientation.rotz = msg.pose.pose.orientation.z;
	orientation.rotw = msg.pose.pose.orientation.w;

	if (state == IDLE && !idle_status_sent) 
	{
		robotDetails.x = pose.px;
		robotDetails.y = pose.py;
		core_alert.publish(robotDetails);
		idle_status_sent = true;
	}

	//orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),
	//orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*
	//orientation.roty-orientation.rotz*orientation.rotz);
	ROS_INFO("/position/x/%f", this->pose.px);
	ROS_INFO("/position/y/%f", this->pose.py);
	ROS_INFO("/orientation/angle/%f", this->orientation.angle);
	//ROS_INFO("/status/%s/./", enum_to_string(state));

	if(localBinStatus.bin_stat == "FULL")
	{
		geometry_msgs::Point location_point;
		location_point.x = localBinStatus.bin_x;
		location_point.y = localBinStatus.bin_y;
		action_queue.push(location_point);
	}

		
	calculateOrientation();
	begin_action_shortest_path(3.0);
	doAngleCheck();
	checkTurningStatus();
	publish();
}


void Carrier::laser_callback(sensor_msgs::LaserScan msg)
{
	for (int i = 0; i < 180; i++)
	{
		//ROS_INFO("swag = %s",msg.ranges[i]);
	}
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number	
}

void Carrier::bin_loc_callback(ugbots_ros::Position pos)
{

}


void Carrier::move_forward(double distance)
{	
	undergoing_task = true;
	speed.linear_x = 2.0;

	double x = distance * cos(pose.theta);
	double y = distance * sin(pose.theta);

	double distance_x = x - pose.px;
	double distance_y = y - pose.py;
	double distance_z = sqrt(pow(distance_x,2) + pow(distance_y,2));

	if(distance_z < 2.000001){
		speed.linear_x = 0.0;
		moving = false;
	}
}

void Carrier::move(){}
void Carrier::stop()
{
	state = IDLE;
	speed.linear_x = 0.0;
	speed.angular_z = 0.0;
}
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
	//node.publish();	

	node.carrier_alert_pub.publish(node.binStatus);

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
