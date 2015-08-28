/**
	Author: UGBots

	Header file for dog.cpp
	It includes declarations for constructors, methods, enums and variables.
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Dog : public Node
{
public:
	Dog();
	Dog(ros::NodeHandle &n);
	enum State { IDLE, WALKING, RUNNING, RANDOMTURN};
	State state;
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void timerCallback(const ros::TimerEvent& e);
	void move();
	void stop();
	void stopTurn();
	void turnRandomly();
	void walk();
	void run();
	void turnLeft();
	void turnRight();
	void turnBack();
	void collisionDetected();
	void checkTurningStatus();
	void setStatus();
	State generateStatus();

	char const* enum_to_string(State t);
};

