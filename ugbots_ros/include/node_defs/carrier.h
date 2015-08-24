#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <ugbots_ros/bin_status.h>

#include <ugbots_ros/robot_details.h>
#include <ugbots_ros/Position.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Carrier : public Node
{
public:
	bool moving;
	bool undergoing_task;

	bool x_completed;
	bool x_started;
	bool y_completed;
	bool y_started;

	double zero_angle;
	double temprad;

	bool idle_status_sent;
	ros::Subscriber sub_ground;
	ros::Subscriber sub_bin;
	ros::Publisher core_alert;
	ugbots_ros::robot_details robotDetails;

	ros::Subscriber carrier_alert;
	ros::Publisher carrier_alert_pub;

	ugbots_ros::bin_status binStatus;
	ugbots_ros::bin_status localBinStatus;


	Carrier();
	Carrier(ros::NodeHandle &n);

	void ground_callback(nav_msgs::Odometry msg);
	void bin_loc_callback(ugbots_ros::Position pos);

	void bin_callback(ugbots_ros::bin_status msg);
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	//void turn(bool clockwise, double desired_angle, double temprad);
	//bool moveX(double distance);
	//bool moveY(double distance);
	//bool begin_action();
	void move_forward(double distance);
	void set_status(int status);
	//void checkTurningStatus();
	void stop();
	bool doubleComparator(double a, double b);

	enum State {IDLE, TRAVELLING, CARRYING, AVOIDING, STOPPED};
	State state;

	State state_array[5] = {IDLE, TRAVELLING, CARRYING, AVOIDING, STOPPED};
	


	char const* enum_to_string(State t);

};
