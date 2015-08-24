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

}

Dog::Dog(ros::NodeHandle &n)
{
	//this->n = n;

	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = 10;
	this->pose.py = 20;
	this->speed.linear_x = 0.0;
	this->speed.max_linear_x = 6.0;
	this->speed.angular_z = 0.0;
	this->orientation.currently_turning = false;

	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Dog::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000, &Dog::laser_callback, this);
	this->sub_list.sub_timer = n.createTimer(ros::Duration(5), &Dog::timerCallback, this);

	//ros::Timer timer = n.createTimer(ros::Duration(10), timerCallback);

	this->state = IDLE;

}

void Dog::odom_callback(nav_msgs::Odometry msg)
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
	ROS_INFO("desired_angle: %f", this->orientation.desired_angle);
	ROS_INFO("orientation_angle: %f", this->orientation.angle);


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
			if ((msg.ranges[a] < 4) && (a > 80) && (a < 100)) {
				ROS_INFO("BACK");
				turnBack();
				break;
			} else if ((msg.ranges[a] < 4) && (a <= 80)) {
				ROS_INFO("TURN LEFT");
				turnLeft();
				break;
			} else if ((msg.ranges[a] < 4) && (a >= 100)){
				ROS_INFO("TURN RIGHT");
				turnRight();
				break;
			}
		}
	}
}

void Dog::timerCallback(const ros::TimerEvent& e){
	if (this->orientation.currently_turning == false){
		setStatus();
	}
}

void Dog::setStatus(){
	state = generateStatus();
	if (state == IDLE){
		stop();
	}else if (state == WALKING){
		walk();
	}else if (state == RUNNING){
		run();
	}else{
		//turn dogs orientation angle randomly
		turnRandomly();
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
void Dog::stopTurn(){ //UNUSED METHOD
	this->orientation.currently_turning = false;
	this->speed.linear_x = 4.0;
	this->speed.angular_z = 0.0;
}

void Dog::turnRandomly(){
	int randNum;
	srand (time(NULL));
	randNum = rand() % 6 + 1; //generate a random number between 1 and 6
	this->orientation.currently_turning = true;
	this->orientation.desired_angle = this->orientation.desired_angle + (randNum);
	this->speed.linear_x = 0.1;
	this->speed.angular_z = 1.0;
}

void Dog::walk(){
	this->speed.linear_x = 1.5;
	this->speed.angular_z = 0.0;
}

void Dog::run(){
	this->speed.linear_x = 6.0;
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

//override checkTurningStatus so that dogs velc
void Dog::checkTurningStatus()
{
	if(this->orientation.currently_turning == true)
	{	
		if(doubleComparator(orientation.angle, orientation.desired_angle))
		{
			this->orientation.currently_turning = false;
			//reset dog's status after turning
			setStatus();
		}
	return;
	}
}

void Dog::collisionDetected(){}

char const* Dog::enum_to_string(State t){
    switch(t){
        case WALKING:
            return "WALKING";
        case RUNNING:
            return "RUNNING";
        case IDLE:
            return "IDLE";
        case RANDOMTURN:
            return "RANDOMTURN";   
        default:
            return "INVALID ENUM";
    }
 }

Dog::State Dog::generateStatus(){
	int randNum;
	srand (time(NULL));
/* generate secret number between 1 and 5: */
	randNum = rand() % 5 + 1;
	if (randNum == 1){
		return IDLE;
	}else if (randNum == 2){
		return WALKING;
	}else if (randNum == 3){
		return RANDOMTURN;
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
