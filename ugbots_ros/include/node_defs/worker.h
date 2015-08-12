#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node.h>

class Worker : public Node
{
public:
	Worker();
	Worker(ros::NodeHandle &n);
	virtual ~Worker() {}
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void move();
	void stop();
	void stopTurn();
	void turnLeft();
	void turnRight();
	void doAngleCheck();
	void collisionDetected();

	void stopTurnStatic();
	void spinOnTheSpot();
	void checkTurningStatus();
	void checkStaticTurningStatus();
	void calculateOrientation();

	enum State { IDLE, PATROLLING, RESPONDING, SAWDOG };
	State state;

	char* enum_to_string(State t);

	bool checkedThisRot;

private:
	void init();
};
