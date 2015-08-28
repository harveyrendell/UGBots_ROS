#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include <ctime>
#include <node_defs/picker.h>


Picker::Picker()
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;
	binPercent = 0;
	binCounter = 0;

	idle_status_sent = false;
}

Picker::Picker(ros::NodeHandle &n)
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	set_status(0);
	binPercent = 0;
	binCounter = 0;

	queueDuplicateCheckAngle = 0.0;
	queueDuplicate = true;
	idle_status_sent = false;
	full_bin_sent = false;
	std::string ns = n.getNamespace();
	ns.erase(ns.begin());
	robotDetails.ns = ns;

	sub_station = n.subscribe<ugbots_ros::picker_row>("station", 1000, &Picker::station_callback, this);
	bin_status_alert = n.subscribe<std_msgs::String>("bin_emptied", 1000, &Picker::bsa_callback, this);
	core_alert = n.advertise<ugbots_ros::robot_details>("/idle_pickers", 1000);
	bin_alert = n.advertise<ugbots_ros::robot_details>("/full_bins", 1000);

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Picker::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Picker::laser_callback, this);
}

//Callback method for when base pose ground truth is published
void Picker::odom_callback(nav_msgs::Odometry msg)
{
	//set up the robots pose and orientation based on the msg
	pose.px = msg.pose.pose.position.x;
	pose.py = msg.pose.pose.position.y;
	orientation.rotx = msg.pose.pose.orientation.x;
	orientation.roty = msg.pose.pose.orientation.y;
	orientation.rotz = msg.pose.pose.orientation.z;
	orientation.rotw = msg.pose.pose.orientation.w;
	orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),
	orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*
	orientation.roty-orientation.rotz*orientation.rotz);

	//when robot idle send its details(x, y, namespace) exactly once
	if (state == IDLE && !idle_status_sent) {
		robotDetails.x = pose.px;
		robotDetails.y = pose.py;
		core_alert.publish(robotDetails);
		idle_status_sent = true;
	}

	//Appropriate ros info to be used by the debugger
	ROS_INFO("/position/x/%f", pose.px);
	ROS_INFO("/position/y/%f", pose.py);
	ROS_INFO("/status/%s/./", enum_to_string(state));

	//if there is avoidance required do not execute action queue
	if(!avoidance_queue.empty())
	{
		//avoidance queue executed
		begin_action_avoidance(3.0);
	}
	else
	{
		//state based speed
		if (state == IDLE) {
			//when idle speed is set to be 0
			speed.linear_x = 0;
			begin_action(0);
		} else if (state == TRAVELLING) {
			//when travelling speed is set to be 0.5
			begin_action(0.5);
		} else if (state == PICKING) {
			//when picking speed is set to be 0.01
			begin_action(0.1);
			//bin starts filling up
			pickKiwi();
		} else if (state == WAITING) {
			//when waiting speed is set to be 0
			speed.linear_x = 0;
			//call the carrier if it hasn't been called yet
			if (!full_bin_sent) {
				callForCarrier();
			}
		} else if (state == STOPPED) {
			//speed is 0 when stopped
			speed.linear_x = 0;
		}
	}

	doAngleCheck();
	checkTurningStatus();
	publish();
}

//call back for laser scan from stage
void Picker::laser_callback(sensor_msgs::LaserScan msg)
{
	//boolean for determining if robot has been detected
	bool robot_detected = false;
	//for all the lasers in specfic area of angles
	for (int i = 76; i < 104; i++) {
		//if any of the angles aren't less than 2.4 in range, break with robot detected being false
		if (msg.ranges[i] < 3) {
			robot_detected = true;
		} else {
			robot_detected = false;
			break;
		}
	}
	//if not detected, and not waiting continue action as usual
	if (!robot_detected) {
		if (state != WAITING) {
			begin_action(0);
		}
	} else {
		//stop if robot is detected
		state = STOPPED;
	}
	//laser detection that gets in way
	/*int min_range = (int)(floor(180 * acos(0.75/2)/M_PI));
	int max_range = (int)(ceil(180 * acos(-0.75/2)/M_PI));

	for (int i = min_range; i < max_range; i++)
	{
		if(msg.ranges[i] < 2.0)
		{
			speed.linear_x = 0.0;
			publish();

			if(!this->orientation.currently_turning)
			{
				while(!avoidance_queue.empty())
				{
					avoidance_queue.pop();
				}

				std::queue<geometry_msgs::Point> temp_queue;

				geometry_msgs::Point pointtemp;
				
				pointtemp.x = this->pose.px + 0.8 * cos(this->orientation.angle - (M_PI/2.0));
				pointtemp.y = this->pose.py + 0.8 * sin(this->orientation.angle - (M_PI/2.0));
				ROS_INFO("first point x: %f",pointtemp.x);
				ROS_INFO("first point y: %f",pointtemp.y);
				avoidance_queue.push(pointtemp);

				pointtemp.x = pointtemp.x + 4 * cos(this->orientation.angle);
				pointtemp.y = pointtemp.y + 4 * sin(this->orientation.angle);
				ROS_INFO("second point x: %f",pointtemp.x);
				ROS_INFO("second point y: %f",pointtemp.y);
				avoidance_queue.push(pointtemp);

				pointtemp.x = pointtemp.x + 0.8 * cos(this->orientation.angle + (M_PI/2.0));
				pointtemp.y = pointtemp.y + 0.8 * sin(this->orientation.angle + (M_PI/2.0));
				ROS_INFO("third point x: %f",pointtemp.x);
				ROS_INFO("third point y: %f",pointtemp.y);				
				avoidance_queue.push(pointtemp);
			}
		}
	}*/
}

//call back for when bin has been alerted to be empty
void Picker::bsa_callback(std_msgs::String msg) {
	//continue action
	begin_action(0);
	//bin is set to empty = 0
	binPercent = 0;
	//bin sent alert is set to false
	full_bin_sent = false;
}

//call back for when the picker robot receives coordinates for work
void Picker::station_callback(ugbots_ros::picker_row pos)
{
	//its start point for picking fruits
	geometry_msgs::Point p;
	p.x = pos.start_x;
	p.y = pos.start_y;
	action_queue.push(p);
	state_queue.push(1);
	//its end point for picking fruits
	p.x = pos.end_x;
	p.y = pos.end_y;
	action_queue.push(p);
	state_queue.push(4);
}

//sets status relative to its element index on the array of status
void Picker::set_status(int status){
	for(int i = 0; i < arraysize(state_array); i++)
	{
		if(i == status)
		{
			state = state_array[i];
			//if just set to idle, change idle_status_sent to be false
			if (state == IDLE) {
				idle_status_sent = false;
			}
		}
	}
}

//function putting robot into picking mode
void Picker::pickKiwi() {
	//if the bin is empty
	if(binPercent<100){
		int num;
		//only increment every multiple of 30 for bin counter
		num = binCounter%30;
		if (num==0) {
			binPercent = binPercent + 1;
		}
		binCounter++;
		ROS_INFO("/bin/%d", binPercent);
		ROS_INFO("/message/the bin is %d percent full", binPercent);

	} else if (binPercent == 100){
		//set bin counter to be 0 and set state as waiting when bin is full
		binPercent = 100;
		ROS_INFO("/bin/%d", binPercent);
		ROS_INFO("/message/the bin is %d percent full", binPercent);
		binCounter = 0;
		state = WAITING;
	}	
}

//method to call for carrier robot when bin is full
void Picker::callForCarrier() {
	//initialise the bins position and the robots namespace
	ugbots_ros::robot_details bin_pos;
	bin_pos.x = pose.px;
	bin_pos.y = pose.py - 2.8;
	bin_pos.ns = robotDetails.ns;;
	bin_alert.publish(bin_pos);
	//bin message sent set to true
	full_bin_sent = true;
}

char const* Picker::enum_to_string(State t) {
	switch (t){
		case IDLE:
			return "IDLE";
		case TRAVELLING:
			return "TRAVELLING";
		case PICKING:
			return "PICKING";
		case WAITING:
			return "WAITING";
		case AVOIDING:
			return "AVOIDING";
		case STOPPED:
			return "STOPPED";
		default:
			return ""; 
	}
}

void Picker::move(){}
void Picker::stop(){
	speed.linear_x = 0.0;
	speed.linear_y = 0.0;
	speed.angular_z = 0.0;

}
void Picker::turnLeft(){}
void Picker::turnRight(){}
void Picker::collisionDetected(){}

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
