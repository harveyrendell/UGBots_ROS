#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Tractor : public Node
{
public:
	Tractor();
	Tractor(ros::NodeHandle &n);
	void ground_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);

	enum State { IDLE, TRAVELLING };
	State state;
	char const* enum_to_string(State t);

	void move();
	void stop();
	void collisionDetected();
};