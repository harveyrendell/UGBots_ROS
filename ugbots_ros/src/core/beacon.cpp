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
	bool isPublished() { return sent; }
	void setX(double x) { this->x = x; }
	void setY(double y) { this->y = y; }
	void setAsPublished() { sent = true; }

	//callback method for the beacons base pose ground truth topic
	void bpgt_callback(nav_msgs::Odometry msg)
	{
		//set the beacons x and y position using base pose ground truth
		setX(msg.pose.pose.position.x);
		setY(msg.pose.pose.position.y);

		//if its position has not been sent
		if (!sent) {
			//publish to the topic /world_layout
			ugbots_ros::Position p;
			p.x = x;
			p.y = y;
			position_pub.publish(p);
			sent = true;
		}
	}

private:
	// coordinates for the beacon
	double x;
	double y;
	//boolean determining position sent status
	bool sent = false;
};

int main(int argc, char **argv)
{
	// initialise node
	ros::init(argc, argv, "BEACON");
	ros::NodeHandle n;

	// construct instance of class
	Beacon b(n);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}

