#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/dog.h>

Dog::Dog()
{
	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 4.0;
	this->speed.max_linear_x = 4.0;
	this->speed.angular_z = 0.0;

	this->orientation.previous_right_distance = 0;
	this->orientation.previous_left_distance = 0;
	this->orientation.previous_front_distance = 0;
	this->orientation.angle = 0;
	this->orientation.desired_angle = M_PI;
	this->orientation.currently_turning = false;

	endOfPath = false;
	facingRight = true;
	facingLeft = false;

	state = RUNNING;	
}

Dog::Dog(ros::NodeHandle &n)
{
	//this->n = n;

	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 4.0;
	this->speed.max_linear_x = 4.0;
	this->speed.angular_z = 0.0;

	this->orientation.previous_right_distance = 0;
	this->orientation.previous_left_distance = 0;
	this->orientation.previous_front_distance = 0;
	this->orientation.angle = 0;
	this->orientation.desired_angle = M_PI;
	this->orientation.currently_turning = false;

	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Dog::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000, &Dog::laser_callback, this);
	this->sub_list.sub_timer = n.createTimer(ros::Duration(5), &Dog::timerCallback, this);

	//ros::Timer timer = n.createTimer(ros::Duration(10), timerCallback);

	endOfPath = false;
	facingRight = true;
	facingLeft = false;

	state = RUNNING;

}

void Dog::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	this->pose.px = -16 + msg.pose.pose.position.x;
	this->pose.py = 42.5 + msg.pose.pose.position.y;

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
	publish();

}


void Dog::laser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	bool detection = false;
	if (this->orientation.currently_turning == false){
		ROS_INFO("11111");
		for(int a = 0 ; a < 180; a++){
			if ((msg.ranges[a] < 5.8) && (a > 85) && (a < 95)) {
				ROS_INFO("22222");
				detection = true;
				turnBack();
				break;
			} else if ((msg.ranges[a] < 5.8) && (a <= 85)) {
				ROS_INFO("33333");
				detection = true;
				turnLeft();
				break;
			} else if ((msg.ranges[a] < 5.8) && (a >= 95)){
				ROS_INFO("44444");
				detection = true;
				turnRight();
				break;
			}
		}
	}
}

void Dog::timerCallback(const ros::TimerEvent& e){
	ROS_INFO("Callback 1 triggered");
	state = generateStatus();
	if (state == IDLE){
		stop();
	}else if (state == WALKING){
		walk();
	}else if (state == RUNNING){
		run();
	}else{
		stop();
	}
}

void Dog::move(){}

//Stops the node
void Dog::stop(){
	this->speed.linear_x = 0.0;
	this->speed.angular_z = 0.0;
}

//Stops the node turning. Linear velocity will be set to default (1.0)
//Update the next desired angle
void Dog::stopTurn(){
	this->orientation.currently_turning = false;
	this->speed.linear_x = 4.0;
	this->speed.angular_z = 0.0;
}

void Dog::walk(){
	this->speed.linear_x = 1.5;
	this->speed.angular_z = 0.0;
}

void Dog::run(){
	this->speed.linear_x = 4.0;
	this->speed.angular_z = 0.0;
}

//Turn left
void Dog::turnLeft(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI / 2);
	this->speed.linear_x = 0.5;
	this->speed.angular_z = 5.0;
}

//Turn right
void Dog::turnRight(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle - (M_PI / 2);
	this->speed.linear_x = 0.5;
	this->speed.angular_z = -5.0;
}

//Turn back
void Dog::turnBack(){
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI);
	this->speed.linear_x = 0.1;
	this->speed.angular_z = 5.0;
}

void Dog::collisionDetected(){}

char const* Dog::enum_to_string(State t){
    switch(t){
        case AGGRESSIVE:
            return "AGGRESSIVE";
        case WALKING:
            return "WALKING";
        case RUNNING:
            return "RUNNING";
        case IDLE:
            return "IDLE";   
        default:
            return "INVALID ENUM";
    }
 }

Dog::State Dog::generateStatus(){
	int randNum;
	srand (time(NULL));
/* generate secret number between 1 and 3: */
	randNum = rand() % 3 + 1;
	if (randNum == 1){
		return IDLE;
	}else if (randNum == 2){
		return WALKING;
	}else{
		return RUNNING;
	}
}

int main(int argc, char **argv)
{	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "DOG");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

Dog dg(n);

//ros::Timer timer = n.createTimer(ros::Duration(10), &Dog::timerCallback);


/*//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
//ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 
//use the one below when using launch, use the one above when testing individual robot
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000); 

//subscribe to listen to messages coming from stage
//robo.setSubs(n);
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &R0::StageOdom_callback, &robo);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&R0::StageLaser_callback, &robo);*/


ros::Rate loop_rate(10);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

while (ros::ok())
{

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
