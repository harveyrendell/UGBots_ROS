#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <string>
#include <stdlib.h>
#include <node.h>

using std::string;

class Dog : public Node
{
public:
	Dog();
	Dog(ros::NodeHandle &n);
	virtual ~Dog() {}
	void odom_callback(nav_msgs::Odometry msg);
	void laser_callback(sensor_msgs::LaserScan msg);
	void move();
	void stop();
	void stopTurn();
	void turnLeft();
	void turnRight();
	void collisionDetected();
	void calculateOrientation();
	void doAngleCheck();

	enum State { IDLE, ROAMING, AGGRESSIVE, FLEEING };
	State state;

	string enum_to_string(State t);

	bool endOfPath;
	bool facingRight;
	bool facingLeft;
	
private:
	void init();
};

