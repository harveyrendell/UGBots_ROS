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

	x_completed = false;
	x_started = false;
	y_completed = false;
	x_started = false;

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_1/base_pose_ground_truth",1000, &Carrier::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan",1000,&Carrier::laser_callback, this);
	carrier_alert = n.subscribe<ugbots_ros::bin_status>("/alert",1000,&Carrier::bin_callback,this);
	carrier_alert_pub = n.advertise<ugbots_ros::bin_status>("/alert",1000);

	geometry_msgs::Point point;
	point.x = 36.0;
	point.y = -4.0;
	action_queue.push(point);
	point.y = 0.0;
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
	orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),
	orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*
	orientation.roty-orientation.rotz*orientation.rotz);
	//ROS_INFO("/position/x/%f", this->pose.px);
	//ROS_INFO("/position/y/%f", this->pose.py);
	//ROS_INFO("/speed/x/%f", msg.twist.twist.linear.x);
	//ROS_INFO("/speed/y/%f", msg.twist.twist.linear.y);
	//ROS_INFO("/status/%s/./", enum_to_string(state));
	ROS_INFO("x should be: %f", pose.px);

	if(localBinStatus.bin_stat == "FULL")
	{
		geometry_msgs::Point location_point;
		location_point.x = localBinStatus.bin_x;
		location_point.y = localBinStatus.bin_y;
		action_queue.push(location_point);
	}

		
	begin_action();
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
	if (orientation.desired_angle - (current_angular_z/10) >= orientation.angle) {
		
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
void Carrier::moveX(double distance) {
	double distance_x = distance - pose.px;
	
	speed.linear_x = 3.0;
	if (distance_x < 0.0 && x_started == false)
	{
		turn(false, M_PI/2 + orientation.angle, 0.0);
		x_started = true;
		return;
	}
	speed.linear_x = 3.0;
	if (abs(distance_x) < 0.1)
	{
		//ROS_INFO("distance_x is: %f", distance_x);
		speed.linear_x = 0.1;
	}

	//ROS_INFO("distance_x: %f", distance_x);
	
	if (distance_x < 0.001) {
		x_completed = true;
		x_started = false;
		stop();
	}
}

void Carrier::moveY(double distance) {
	double distance_y = distance - pose.py;

	
	if (distance_y < 0.0 && y_started == false)
	{
		ROS_INFO("distance_x: %f", distance_y);
		ROS_INFO("orientation: %f", M_PI + orientation.angle);
		turn(false, M_PI + orientation.angle, 0.0);
		return;
	}
	speed.linear_x = 3.0;
	if (abs(distance_y) < 0.1)
	{
		speed.linear_x = 0.1; 
	}
	if (distance_y < 0.001) {
		x_completed = false;
		y_started = false;
		stop();
	}
}

bool Carrier::begin_action()
{

	if (action_queue.empty())
	{
		state = IDLE;
		return true;
	}
	geometry_msgs::Point end_point = action_queue.front();
	if(doubleComparator(end_point.x, pose.px) && doubleComparator(end_point.y, pose.py))
	{
		action_queue.pop();
		stop();
	}
	state = TRAVELLING;
	if(!x_completed)
	{
		moveX(end_point.x);
	}
	else
	{
		turn(false, M_PI/2, 0.0);
		moveY(end_point.y);
	}

	


/*
	moveX(end_point.x);
	if (speed.linear_x == 0.0) 
	{
		//turn(double angle, double linear, double angular)
		turn(false, M_PI/2, 0.0);
		if (speed.angular_z == 0.0)
		{
			state = TRAVELLING;
			speed.linear_x = 3.0;
			moveY(end_point.y);
			temprad = orientation.angle;
		}	
	}**/
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

bool Carrier::doubleComparator(double a, double b)
{
    return abs(a - b) < 0.001;
}

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
