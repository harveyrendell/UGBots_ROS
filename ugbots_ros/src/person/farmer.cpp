#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include "../Headers/Unit.h"

class Farmer : public Unit
{
public:
	Farmer(ros::NodeHandle &n)
	{
		this->n = n;

		//setting base attribute defaults
		pose.theta = M_PI/2.0;
		pose.px = 10;
		pose.py = 20;
		speed.linear_x = 0.0;
		speed.max_linear_x = 3.0;
		speed.angular_z = 20.0;

		sub_list.node_stage_pubb = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
		sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, &Farmer::odom_callback, this);
		sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Farmer::laser_callback, this);
	}
};
