#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ugbots_ros/bin_status.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Picker : public Node
{
public:
	ros::Publisher carrier_alert;
	ugbots_ros::bin_status binStatus;

	Picker(ros::NodeHandle &n);
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void move();
	void stop();
	void turnLeft();
	void turnRight();
	void collisionDetected();

	enum State { IDLE, TRAVELLING, PICKING, WAITING, AVOIDING, STOPPED };
	State state;
};