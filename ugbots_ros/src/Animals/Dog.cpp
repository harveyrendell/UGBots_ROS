#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include "Robot.h"
#include "Unit.h"

class Dog : public Animal
{
public:
	Dog(ros::NodeHandle &n);
	void
};
Dog::Dog(ros::NodeHandle &n): Animal(&n)
