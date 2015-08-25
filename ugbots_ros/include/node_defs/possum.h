#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Possum : public Node
{
public:
	Possum();
	Possum(ros::NodeHandle &n);
	enum State { IDLE, ROAMING, FLEEING, MOVINGACROSS };
	State state;
	int row;
	enum Direction {NORTH, EAST, SOUTH, WEST};
	Direction direction;
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void timerCallback(const ros::TimerEvent& e);
	void move();
	void stop();
	void stopTurn();
	void turnLeft();
	void turnRight();
	void turnBack();
	void checkTurningStatus();
	void walk();
	void run();
	void collisionDetected();
	State generateStatus();
	char const* enum_to_string(State t);
};
