#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/possum.h>

Possum::Possum()
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;

	this->orientation.previous_right_distance = 0;
	this->orientation.previous_left_distance = 0;
	this->orientation.previous_front_distance = 0;
	this->orientation.angle = 0;
	this->orientation.desired_angle = M_PI;
	this->orientation.currently_turning = false;

	state = IDLE;
}

Possum::Possum(ros::NodeHandle &n)
{
	//this->n = n;

	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;

	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Possum::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Possum::laser_callback, this);
	this->sub_list.sub_timer = n.createTimer(ros::Duration(5), &Possum::timerCallback, this);

	state = IDLE;
}

void Possum::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	this->pose.px = 48 + msg.pose.pose.position.x;
	this->pose.py = 48 + msg.pose.pose.position.y;

	this->orientation.rotx = msg.pose.pose.orientation.x;
	this->orientation.roty = msg.pose.pose.orientation.y;
	this->orientation.rotz = msg.pose.pose.orientation.z;
	this->orientation.rotw = msg.pose.pose.orientation.w;

	ROS_INFO("/position/x/%f", this->pose.px);
	ROS_INFO("/position/y/%f", this->pose.py);
	ROS_INFO("/status/%s/./", enum_to_string(state));


	calculateOrientation();
	doAngleCheck();
	checkTurningStatus();

}

void Possum::laser_callback(sensor_msgs::LaserScan msg)
{
	if (this->orientation.currently_turning == false){
		if ((msg.ranges[90] <= 2) && (msg.ranges[179] <= 2)){
			turnRight();
		}else if ((msg.ranges[90] <= 2) && (msg.ranges[0] <= 2)){
			turnLeft();
		}
	}
}

void Possum::timerCallback(const ros::TimerEvent& e){
	ROS_INFO("Callback 1 triggered");
	state = generateStatus();
	if (state == IDLE){
		stop();
	}else if (state == ROAMING){
		walk();
	}else if (state == FLEEING){
		run();
	}else{
		stop();
	}
}

void Possum::move(){

}
void Possum::stop(){
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 0.0;
}
void Possum::stopTurn(){
	this->orientation.currently_turning = false;
	this->speed.linear_x = 4.0;
	this->speed.angular_z = 0.0;
}
void Possum::walk(){
	this->speed.linear_x = 1.5;
	this->speed.angular_z = 0.0;
}

void Possum::run(){
	this->speed.linear_x = 6.0;
	this->speed.angular_z = 0.0;
}
void Possum::turnLeft(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI / 2);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 5.0;
}
void Possum::turnRight(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle - (M_PI / 2);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = -5.0;
}
//Turn back
void Possum::turnBack(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 5.0;
}
void Possum::calculateOrientation()
{	
	this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);
}

//Angle translation for easier interpretation
void Possum::doAngleCheck(){		
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

void Possum::checkTurningStatus()
{
	if(this->orientation.currently_turning == true)
	{
		if((this->orientation.angle + (M_PI / (speed.angular_z * 2) ) ) >= this->orientation.desired_angle)
		{
			stopTurn(); // stop the turn when desired angle is reacahed (2 clocks before the estimated angle)
		}
		return;
	}
}

char* Possum::enum_to_string(State t){
    switch(t){
        case IDLE:
            return "IDLE";
        case ROAMING:
            return "ROAMING";
        case FLEEING:
            return "FLEEING";   
        default:
            return "INVALID ENUM";
    }
 }

Possum::State Possum::generateStatus(){
	int randNum;
	srand (time(NULL));
/* generate secret number between 1 and 3: */
	randNum = rand() % 3 + 1;
	if (randNum == 1){
		return IDLE;
	}else if (randNum == 2){
		return ROAMING;
	}else{
		return FLEEING;
	}
}

void Possum::collisionDetected(){}

int main(int argc, char **argv)
{	
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "POSSUM");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Possum node(n);

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