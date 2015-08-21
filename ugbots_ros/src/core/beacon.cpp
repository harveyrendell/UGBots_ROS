#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ugbots_ros/Position.h>
#include <nav_msgs/Odometry.h>
#include <node_structs/point.h>

#include <sstream>

class Beacon
{
public:
	// default constructor
	Beacon(ros::NodeHandle &n)
	{
		world_postion = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Beacon::bpgt_callback, this);
		position_pub = n.advertise<ugbots_ros::Position>("/world_layout", 1000);
	}

	// position publisher
	ros::Publisher position_pub;
	ros::Subscriber world_postion;

	// getter methods for x, y
	double getX(){ return x; }
	double getY(){ return y; }
	void setX(double x) { this->x = x; }
	void setY(double y) { this->y = y; }
	void bpgt_callback(nav_msgs::Odometry msg)
	{
		setX(msg.pose.pose.position.x);
		setY(msg.pose.pose.position.y);
	}

private:
	// coordinates for the beacon
	double x;
	double y;
};

int main(int argc, char **argv)
{
	// initialise node
	ros::init(argc, argv, "BEACON");
	ros::NodeHandle n;

	// construct instance of class
	Beacon b(n);

	ros::Rate loop_rate(1);

	int count = 0;
	while (ros::ok())
	{
		// create the msg to publish
		ugbots_ros::Position msg;
		msg.x = b.getX();
		msg.y = b.getY();

		if (count > 1) {
			// publish it
			b.position_pub.publish(msg);
		}

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}

