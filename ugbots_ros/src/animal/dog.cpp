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

	state = ROAMING;	
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

	endOfPath = false;
	facingRight = true;
	facingLeft = false;

	state = ROAMING;

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
	this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);

	ROS_INFO("/position/x/%f", this->pose.px);
	ROS_INFO("/position/y/%f", this->pose.py);
	ROS_INFO("/status/%s/./", enum_to_string(state));


	calculateOrientation();

	doAngleCheck();

	//ROS_INFO("Lets check the angle : %f", this->orientation.angle);

	if ((msg.pose.pose.position.x + 1.1 >= 32) && (facingRight == true)){
		
		endOfPath = true;

		this->speed.linear_x = 0.0;
		this->speed.angular_z = 3.0;
		this->orientation.currently_turning = true;

		if((this->orientation.angle + (M_PI / (this->speed.angular_z * 3))) >= this->orientation.desired_angle){
			this->speed.angular_z = 0.0;// stop the turn when desired angle is reacahed (2 clocks before the estimated angle)
			facingRight = false;
			facingLeft = true;
			endOfPath = false;
			this->orientation.currently_turning = false;
		}
	}

	if ((msg.pose.pose.position.x - 1.0 <= 0) && (facingLeft == true)) {

		endOfPath = true;

		this->speed.linear_x = 0.0;
		this->speed.angular_z = 3.0;
		this->orientation.currently_turning = true;

		if((this->orientation.angle + (M_PI / (this->speed.angular_z * 3))) >= (M_PI * 2)){
			this->speed.angular_z = 0.0;// stop the turn when desired angle is reacahed (2 clocks before the estimated angle)
			facingLeft = false;
			facingRight = true;
			endOfPath = false;
			this->orientation.currently_turning = false;
		}
	}

}


void Dog::laser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	bool detection = false;
	for(int a = 0 ; a < 180; a++){
		if (msg.ranges[a] < 5.8) {
			detection = true;
			continue;
		} 
	}

	if (detection == true){
		this->speed.linear_x = 0.0;
		this->speed.angular_z = 0.0;
		state = AGGRESSIVE;

	} else {
		state = ROAMING;
		if (endOfPath == false){
			this->speed.linear_x = 4.0;
			if (this->orientation.currently_turning == true){
				this->speed.angular_z = 3.0;
			}
		}
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
	this->speed.linear_x = 1.0;
	this->speed.angular_z = 0.0;
	this->orientation.desired_angle = this->orientation.desired_angle + (M_PI / 2.000000);
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

void Dog::calculateOrientation()
{	
	this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);
}

//Angle translation for easier interpretation
void Dog::doAngleCheck(){		
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

void Dog::collisionDetected(){}

char const* Dog::enum_to_string(State t){
    switch(t){
        case AGGRESSIVE:
            return "AGGRESSIVE";
        case ROAMING:
            return "ROAMING";
        case IDLE:
            return "IDLE";
        case FLEEING:
            return "FLEEING";   
        default:
            return "INVALID ENUM";
    }
 }

int main(int argc, char **argv)
{	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "DOG");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

Dog dg(n);

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

	//messages to stage
	RobotNode_cmdvel.linear.x = dg.speed.linear_x;
	RobotNode_cmdvel.angular.z = dg.speed.angular_z;
        
	//publish the message
	dg.sub_list.node_stage_pub.publish(RobotNode_cmdvel);

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
