#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/picker.h>


Picker::Picker(ros::NodeHandle &n)
{
	this->n = n;

	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 1.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, &Picker::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,&Picker::laser_callback, this);
	carrier_alert = n.advertise<ugbots_ros::bin_status>("alert",1000);
}

/*void Picker::logic() {
	if (turningLeft) {
		turn(false, M_PI/2);
	} else if (turningRight) {
		turn(true, M_PI/2);
	}

	if (stopped) {
		speed.linear_x = 0.0;
	} else {
		speed.linear_x = 1.0;
	}
}*/

/*void Picker::turn(bool clockwise, double desired_angle) {
	double current_angular_z;

	//desired angle of turn added to robots current angle facing
	orientation.desired_angle = desired_angle + zero_angle;

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
	if (orientation.desired_angle-2*(current_angular_z/10) >= orientation.angle) {
	//if desired angle is reached, robot stops turning and moves again 
	} else {
		orientation.currently_turning = false;
		//stopped = false;
		speed.angular_z = 0.0;
		zero_angle = orientation.desired_angle;
	}
}*/
void Picker::turn(bool clockwise, double desired_angle, double temprad) {
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
		orientation.currently_turning = false;
		//stopped = false;
		speed.angular_z = 0.0;
		zero_angle = orientation.desired_angle;
	}
}

void Picker::moveX(double distance, double px) {
	double x = distance + px;
	double distance_x = x - pose.px;
	if (distance_x < 0.20001) {
		speed.linear_x = 0.0;
	}
}

void Picker::moveY(double distance, double py) {
	double y = distance + py;
	double distance_y = y - pose.py;
	if (distance_y < 0.20001) {
		speed.linear_x = 0.0;
	}
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
	pose.px = -10 + msg.pose.pose.position.x;
	pose.py = -40 + msg.pose.pose.position.y;
	ROS_INFO("Current x position is: %f", pose.px);
	ROS_INFO("Current y position is: %f", pose.py);
	orientation.rotx = msg.pose.pose.orientation.x;
	orientation.roty = msg.pose.pose.orientation.y;
	orientation.rotz = msg.pose.pose.orientation.z;
	orientation.rotw = msg.pose.pose.orientation.w;
	orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),
		orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*
		orientation.roty-orientation.rotz*orientation.rotz);
	ROS_INFO("Current angle is: %f", orientation.angle);

	//bin location, currently attached to the centre of robot
	binStatus.bin_x = pose.px;
	binStatus.bin_y = pose.py;

	//relative actions for different states
	if (state == IDLE) {
		state = TRAVELLING;
		tempx = pose.px;
		tempy = pose.py;
		temprad = orientation.angle;
		goToWork();
		state = TRAVELLING;
	} else if (state == TRAVELLING) {
		goToWork();
	} else if (state == PICKING) {
		pickKiwi();
	} else if (state == WAITING) {
		binStatus.bin_x = 0.0;
		binStatus.bin_y = binStatus.bin_y-1.5;
		binStatus.bin_stat = "FULL";
	}

	//publish topic about current bin status
	carrier_alert.publish(binStatus);
}


void Picker::laser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

//hard coded function for robot to get to work station
void Picker::goToWork() {
	moveX(abs(station_x-tempx),tempx);
	if (speed.linear_x == 0.0) {
		turn(false, M_PI/2, temprad);
		if (speed.angular_z == 0.0){
			speed.linear_x = 1.0;
			moveY(abs(station_y-tempy),tempy);
			if (pose.py > -35.0){
				state = PICKING;
				tempx = pose.px;
				tempy = pose.py;
				temprad = orientation.angle;
			}
		}
	}
}

//function putting robot into picking mode
void Picker::pickKiwi() {
	speed.linear_x = 0.5;
	binStatus.bin_stat = "FILLING";
	moveY(70.0,tempy);
	if (speed.linear_x == 0.0) {
		state = WAITING;
	}
}

void Picker::move(){}
void Picker::stop(){}
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
