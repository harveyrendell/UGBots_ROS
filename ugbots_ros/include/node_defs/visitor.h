#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ugbots_ros/Position.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Visitor : public Node
{
public:
	Visitor();
	Visitor(ros::NodeHandle &n);
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void core_callback(ugbots_ros::Position msg);
	void move();
	void stop();
	void turnLeft();
	void turnRight();
	void collisionDetected();
	void checkTurningStatus();
	void init_route();

	enum State { IDLE, LOITERING, HARASSING, STOPPED };
	State state;

	bool rightTurnInit;
	bool leftTurnInit;
	bool moveToEnabled;
	double queueDuplicate;
	double queueDuplicateCheckAngle;
	ros::Subscriber sub_row;

	char const* enum_to_string(State t);
};