#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Cat : public Node
{
public:
	Cat();
	Cat(ros::NodeHandle &n);
	enum State { IDLE, ROAMING, RUNNING};
	State state;
	enum Direction {CLOCKWISE, ANTICLOCKWISE};
	Direction direction;
	enum Position {NORTH, EAST, SOUTH, WEST};
	Position position;
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void timerCallback(const ros::TimerEvent& e);
	void move();
	void stopAfterPop();
	void stop();
	void stopTurn();
	void turnLeft();
	void turnRight();
	void turnBack();
	void checkTurningStatus();
	void walk();
	void run();
	void collisionDetected();
	//void checkTurningStatus();
	State generateStatus();
	char* enum_to_string(State t);
	bool begin_action(double speed);

};
