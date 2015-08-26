#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include <node_defs/picker.h>


Picker::Picker()
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;
	station_x = 0;
	station_y = -33;
	binPercent = 0;

	idle_status_sent = false;
}

Picker::Picker(ros::NodeHandle &n)
{
	//this->n = n;

	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	set_status(0);
	station_y = -33;

	queueDuplicateCheckAngle = 0.0;
	queueDuplicate = true;
	idle_status_sent = false;
	std::string ns = n.getNamespace();
	ns.erase(ns.begin());
	robotDetails.ns = ns;

	sub_station = n.subscribe<ugbots_ros::picker_row>("station", 1000, &Picker::station_callback, this);
	core_alert = n.advertise<ugbots_ros::robot_details>("/idle_pickers", 1000);
	bin_alert = n.advertise<ugbots_ros::Position>("/full_bins", 1000);

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_13/cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_13/base_pose_ground_truth",1000, &Picker::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_13/base_scan",1000,&Picker::laser_callback, this);
	carrier_alert = n.advertise<ugbots_ros::bin_status>("/alert",1000);

}

void Picker::move(double distance, double px, double py)
{
	double x = distance * sin(pose.theta) + px;
	double y = distance * cos(pose.theta) + py;

	double distance_x = x - pose.px;
	double distance_y = y - pose.py;
	double distance_z = sqrt(pow(distance_x,2) + pow(distance_y,2));

	if(distance_z < 0.20001)
	{
		speed.linear_x = 0.0;
	}
}

void Picker::odom_callback(nav_msgs::Odometry msg)
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

	if (state == IDLE && !idle_status_sent) {
		robotDetails.x = pose.px;
		robotDetails.y = pose.py;
		ROS_INFO("x: %f, y: %f", robotDetails.x, robotDetails.y);
		core_alert.publish(robotDetails);
		idle_status_sent = true;
	}

	ROS_INFO("/position/x/%f", action_queue.front().x);
	ROS_INFO("/position/y/%f", action_queue.front().y);
	ROS_INFO("/status/%s/./", enum_to_string(state));
	//ROS_INFO("%f degrees per clock", (msg.twist.twist.angular.z * 180/M_PI)/10);


	//bin location, currently attached to the centre of robot
	binStatus.bin_x = pose.px;
	binStatus.bin_y = pose.py;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	station_x = 0;


	/*if(action_queue.empty())
	{
		point.y = 38.0;
		point.x = 10.5;
		action_queue.push(point);
		point.y = -38.0;
		point.x = 10.5;
		action_queue.push(point);
	}*/

	ROS_INFO("point x: %f point y: %f", action_queue.front().x, action_queue.front().y);

	//relative actions for different states

	/*if (state == IDLE) {
		state = TRAVELLING;
		tempx = pose.px;
		tempy = pose.py;
		//temprad = orientation.angle;
		//goToWork();
		//state = TRAVELLING;
	} else if (state == TRAVELLING) {
		goToWork();
	} else if (state == PICKING) {
		pickKiwi();
	} else if (state == WAITING) {
		binStatus.bin_x = 0.0;
		binStatus.bin_y = pose.py-2;
		speed.angular_z = M_PI;
	}**/

	//publish topic about current bin status
	carrier_alert.publish(binStatus);

	if(state == AVOIDING)
	{
		begin_action_avoidance(3.0);
	}
	else
	{
		begin_action_shortest_path(3.0);

		ROS_INFO("in the else loop %f", speed.angular_z);
	}

	doAngleCheck();
	checkTurningStatus();
	ROS_INFO("x: %f, y: %f", action_queue.front().x, action_queue.front().y);
	publish();
}


void Picker::laser_callback(sensor_msgs::LaserScan msg)
{
	//laser detection that gets in way
	int min_range = (int)(floor(180 * acos(0.75/3)/M_PI));
	int max_range = (int)(ceil(180 * acos(-0.75/3)/M_PI));

	for (int i = min_range; i < max_range; i++)
	{
		if(msg.ranges[i] < 3.0)
		{
			speed.linear_x = 0.0;
			publish();
			if(this->queueDuplicate && !this->orientation.currently_turning)
			{
				this->queueDuplicateCheckAngle = this->orientation.angle;

				std::queue<geometry_msgs::Point> temp_queue;

				geometry_msgs::Point pointtemp;
				
				pointtemp.x = this->pose.px + 2 * cos(this->orientation.angle - (M_PI/2.0));
				pointtemp.y = this->pose.py + 2 * sin(this->orientation.angle - (M_PI/2.0));
				ROS_INFO("first point x: %f",pointtemp.x);
				ROS_INFO("first point y: %f",pointtemp.y);
				avoidance_queue.push(pointtemp);

				pointtemp.x = pointtemp.x + 4 * cos(this->orientation.angle);
				pointtemp.y = pointtemp.y + 4 * sin(this->orientation.angle);
				ROS_INFO("second point x: %f",pointtemp.x);
				ROS_INFO("second point y: %f",pointtemp.y);
				avoidance_queue.push(pointtemp);

				pointtemp.x = this->pose.px + 2 * cos(this->orientation.angle + (M_PI/2.0));
				pointtemp.y = this->pose.py + 2 * sin(this->orientation.angle + (M_PI/2.0));
				ROS_INFO("third point x: %f",pointtemp.x);
				ROS_INFO("third point y: %f",pointtemp.y);				
				avoidance_queue.push(pointtemp);

				this->queueDuplicate = false;
			}
		}
	}



	/*if(fabs(this->queueDuplicateCheckAngle - this->orientation.angle) >= (M_PI/2.000000))
	{
		this->queueDuplicate = true;
		this->queueDuplicateCheckAngle = 0;
	}
	

	if(msg.ranges[90] < 2.0)
	{
		if(this->queueDuplicate == true)
		{
			this->queueDuplicateCheckAngle = this->orientation.angle;

			std::queue<geometry_msgs::Point> temp_queue;

			geometry_msgs::Point pointtemp;

			
			pointtemp.x = this->pose.px + sqrt(0.5) * cos(this->orientation.angle - (M_PI/4.0));
			pointtemp.y = this->pose.py + sqrt(0.5) * sin(this->orientation.angle - (M_PI/4.0));
			temp_queue.push(pointtemp);

			pointtemp.x = pointtemp.x + 4 * cos(this->orientation.angle);
			pointtemp.y = pointtemp.y + 4 * sin(this->orientation.angle);
			temp_queue.push(pointtemp);

			pointtemp.x = pointtemp.x + sqrt(0.5) * cos(this->orientation.angle + (M_PI/4.0));
			pointtemp.y = pointtemp.y + sqrt(0.5) * sin(this->orientation.angle + (M_PI/4.0));
			temp_queue.push(pointtemp);

			while(!action_queue.empty())
			{
				temp_queue.push(action_queue.front());
				action_queue.pop();
			}

			while(!temp_queue.empty())
			{
				action_queue.push(temp_queue.front());
				temp_queue.pop();
			}

			this->queueDuplicate = false;
		}
	}**/
}
void Picker::set_status(int status){
	for(int i = 0; i < arraysize(state_array); i++)
	{
		if(i == status)
		{
			state = state_array[i];
			if (state == IDLE) {
				idle_status_sent = false;
			}
		}
	}
}

void Picker::station_callback(ugbots_ros::picker_row pos)
{
	ROS_INFO("Robot given coordinates sx: %f, sy: %f, ex: %f, ey: %f", pos.start_x, pos.start_y, pos.end_x, pos.end_y);
	geometry_msgs::Point p;
	p.x = pos.start_x;
	p.y = pos.start_y;
	action_queue.push(p);
	state_queue.push(1);
	p.x = pos.end_x;
	p.y = pos.end_y;
	action_queue.push(p);
	state_queue.push(2);
}

//hard coded function for robot to get to work station
void Picker::goToWork() {
	/*moveX(abs(station_x-tempx),tempx);
	if (speed.linear_x == 0.0) {
		turn(false, M_PI/2, temprad);
		if (speed.angular_z == 0.0){
			speed.linear_x = 1.0;
			moveY(abs(station_y-tempy),tempy);
			state = PICKING;
			tempx = pose.px;
			tempy = pose.py;
			temprad = orientation.angle;
		}
	}**/
}

//function putting robot into picking mode
void Picker::pickKiwi() {
	speed.linear_x = 0.5;
	
	
	if(binPercent<100){
		int randomInt = rand() % 13;
		if (randomInt == 0) { 
			binPercent = binPercent + 1;
		}
		ROS_INFO("/bin/%d", binPercent);
		ROS_INFO("/message/the bin is %d percent full", binPercent);
		binStatus.bin_stat = "FILLING";

		move_y(70.0,tempy);

	}	


	else if (binPercent == 100){
		binPercent = 100;
		ROS_INFO("/bin/%d", binPercent);
		ROS_INFO("/message/the bin is %d percent full", binPercent);
		binStatus.bin_stat = "FULL";
		state = WAITING;
	}	
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
		default:
			return ""; 
	}
}

void Picker::move(){}
void Picker::stop(){

	state = STOPPED;
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

node.binStatus.bin_stat = "EMPTY";

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

while (ros::ok())
{
	node.publish();

	node.carrier_alert.publish(node.binStatus);

	ros::spinOnce();

	loop_rate.sleep();

	++count;
}
return 0;

}
