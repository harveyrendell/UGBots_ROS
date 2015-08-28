#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ugbots_ros/bin_status.h>

#include <ugbots_ros/picker_row.h>
#include <ugbots_ros/Position.h>
#include <ugbots_ros/robot_details.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Picker : public Node
{
public:
	int binCounter;
	bool idle_status_sent;
	bool full_bin_sent;

	ros::Publisher core_alert;
	ros::Publisher bin_alert;
	ros::Subscriber sub_station;
	ros::Subscriber bin_status_alert;
	ugbots_ros::robot_details robotDetails;

	void station_callback(ugbots_ros::picker_row pos);

	geometry_msgs::Point point;

	double queueDuplicate;
	double queueDuplicateCheckAngle;


	Picker();
	Picker(ros::NodeHandle &n);
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void bsa_callback(std_msgs::String msg);
	void move();
	void stop();
	void turnLeft();
	void turnRight();
	void collisionDetected();

	void set_status(int status);
	
	void pickKiwi();
	void callForCarrier();

	enum State { IDLE, TRAVELLING, AVOIDING, PICKING, WAITING, STOPPED };
	State state;

	State state_array[6] = { IDLE, TRAVELLING, AVOIDING, WAITING, PICKING, STOPPED };

	char const* enum_to_string(State t);
	
	int binPercent;
};
