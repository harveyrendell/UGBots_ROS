#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ugbots_ros/bin_status.h>
#include <ugbots_ros/robot_details.h>
#include <ugbots_ros/Position.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Picker : public Node
{
public:
	double tempx;
	double tempy;
	double temprad;
	double station_x;
	double station_y;
	double zero_angle;

	bool sent;

	ros::Publisher carrier_alert;
	ros::Publisher core_alert;
	ros::Publisher bin_alert;
	ros::Subscriber sub_ground;
	ros::Subscriber station_sub;
	ugbots_ros::bin_status binStatus;
	ugbots_ros::robot_details robotDetails;

	Picker();
	Picker(ros::NodeHandle &n);

	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void ground_callback(nav_msgs::Odometry msg);
	void station_callback(ugbots_ros::Position pos);
	void turn(bool clockwise, double desired_angle, double temprad);
	void move(double distance, double px, double py);
	void moveX(double distance, double px);
	void moveY(double distance, double py);
	void move();
	void stop();
	void turnLeft();
	void turnRight();
	void collisionDetected();
	
	void goToWork();
	void pickKiwi();

	enum State { IDLE, TRAVELLING, PICKING, WAITING, AVOIDING, STOPPED };
	State state;

	char const* enum_to_string(State t);
};