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
	double tempx;
	double tempy;
	double temprad;
	double station_x;
	double station_y;
	double zero_angle;

	bool idle_status_sent;

	ros::Publisher core_alert;
	ros::Publisher bin_alert;
	ros::Subscriber sub_station;
	ugbots_ros::robot_details robotDetails;

	void station_callback(ugbots_ros::picker_row pos);

	ros::Publisher carrier_alert;
	ugbots_ros::bin_status binStatus;

	geometry_msgs::Point point;

	double queueDuplicate;
	double queueDuplicateCheckAngle;


	Picker();
	Picker(ros::NodeHandle &n);
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void turn(bool clockwise, double desired_angle, double temprad);
	void move(double distance, double px, double py);
	void move();
	void stop();
	void turnLeft();
	void turnRight();
	void collisionDetected();

	void set_status(int status);
	
	void goToWork();
	void pickKiwi();

	enum State { IDLE, TRAVELLING, PICKING, WAITING, AVOIDING, STOPPED };
	State state;
	char const* enum_to_string(State t);
	
	int binPercent;
};
