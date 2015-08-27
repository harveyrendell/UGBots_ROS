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
	void doRouteSetup();

	void waiting();
	void startTour();
	bool insideFarm();

	enum State { IDLE, AVOIDING, TOURING };
	State state;

	bool rightTurnInit;
	bool leftTurnInit;
	bool moveToEnabled;
	bool waitingInLine;
	bool tourStarted;
	bool angleChangeStarted;
	double queueDuplicate;
	double queueDuplicateCheckAngle;
	ros::Subscriber sub_row;
	
	//Beacon points
	std::list<geometry_msgs::Point> beacon_points;


	char const* enum_to_string(State t);
};