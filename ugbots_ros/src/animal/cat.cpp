#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/cat.h>

Cat::Cat()
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;
	

	state = IDLE;
}

Cat::Cat(ros::NodeHandle &n)
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
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Cat::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Cat::laser_callback, this);
	this->sub_list.sub_timer = n.createTimer(ros::Duration(5), &Cat::timerCallback, this);

	state = IDLE;


	geometry_msgs::Point point;
	point.x = 48.0;
	point.y = 48.0;
	action_queue.push(point);
	point.x = 48.0;
	point.y = -48.0;
	action_queue.push(point);
	point.x = -48.0;
	point.y = -48.0;
	action_queue.push(point);
	point.x = -48.0;
	point.y = 48.0;
	action_queue.push(point);
}

void Cat::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	this->pose.px = msg.pose.pose.position.x;
	this->pose.py = msg.pose.pose.position.y;

	this->orientation.rotx = msg.pose.pose.orientation.x;
	this->orientation.roty = msg.pose.pose.orientation.y;
	this->orientation.rotz = msg.pose.pose.orientation.z;
	this->orientation.rotw = msg.pose.pose.orientation.w;

	ROS_INFO("/position/x/%f", this->pose.px);
	ROS_INFO("/position/y/%f", this->pose.py);
	ROS_INFO("/status/%s/./", enum_to_string(state));
	ROS_INFO("linear speed: %f", this->speed.linear_x);
	ROS_INFO("angular speed: %f", this->speed.angular_z);
	ROS_INFO("desired_angle: %f", this->orientation.desired_angle);
	ROS_INFO("orientation_angle: %f", this->orientation.angle);
	ROS_INFO("%f, %f", action_queue.front().x , action_queue.front().y);
	if (this->orientation.currently_turning == true){
		ROS_INFO("TRUE");
	}else {
		ROS_INFO("FALSE");
	}

	calculateOrientation();
	begin_action(this->speed.linear_x);
	doAngleCheck();
	checkTurningStatus();
	publish();

}

void Cat::laser_callback(sensor_msgs::LaserScan msg)
{
	
}

void Cat::timerCallback(const ros::TimerEvent& e){
	if (this->orientation.currently_turning == false){
		state = generateStatus();
		if (state == IDLE){
			stop();
		}else if (state == ROAMING){
			walk();
		}else{
			run();
		}
	}
}

void Cat::move(){

}
void Cat::stop(){
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 0.0;
}
void Cat::stopTurn(){
	this->orientation.currently_turning = false;
	this->speed.linear_x = 4.0;
	this->speed.angular_z = 0.0;
}
void Cat::walk(){
	this->speed.linear_x = 1.5;
	this->speed.angular_z = 0.0;
}

void Cat::run(){
	this->speed.linear_x = 6.0;
	this->speed.angular_z = 0.0;
}
void Cat::turnLeft(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI / 2);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 5.0;
}
void Cat::turnRight(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle - (M_PI / 2);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = -5.0;
}
//Turn back
void Cat::turnBack(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI);
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 5.0;
}

char* Cat::enum_to_string(State t){
    switch(t){
        case IDLE:
            return "IDLE";
        case ROAMING:
            return "ROAMING";
        case RUNNING:
            return "RUNNING";  
        default:
            return "INVALID ENUM";
    }
 }

Cat::State Cat::generateStatus(){
	int randNum;
	srand (time(NULL));
/* generate secret number between 1 and 3: */
	randNum = rand() % 4 + 1;
	if (randNum == 1){
		return IDLE;
	}else if (randNum == 2){
		return ROAMING;
	} else {
		return RUNNING;
	}
}

void Cat::collisionDetected(){} 

int main(int argc, char **argv)
{	
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "CAT");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Cat node(n);

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

while (ros::ok())
{
	//node.publish();
	
	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}