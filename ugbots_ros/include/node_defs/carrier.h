#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ugbots_ros/bin_status.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Carrier : public Node
{
public:
	bool moving = false;
	bool undergoing_task = false;
	double tempx;
	double tempy;
	bool swag = false;
	double zero_angle;
	double temprad;

	ros::Subscriber carrier_alert;


	Carrier(ros::NodeHandle &n);

	void bin_callback(ugbots_ros::bin_status msg);
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void turn(bool clockwise, double desired_angle, double temprad);
	void moveX(double distance, double px);
	void moveY(double distance, double py);
	bool move_to(double x, double y);
	void move_forward(double distance);
	void move();
	void stop();
	void turnLeft();
	void turnRight();
	void collisionDetected();

	enum State { IDLE, TRAVELLING, CARRYING, AVOIDING, STOPPED };
	State state;
};