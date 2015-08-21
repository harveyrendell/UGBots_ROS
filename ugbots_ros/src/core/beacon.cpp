#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ugbots_ros/Position.h>

#include <sstream>

class Beacon
{
public:
	// default constructor
	Beacon(){}

	// position publisher
	ros::Publisher position_pub;

	// getter methods for x, y
	double getX(){ return x; }
	double getY(){ return y; }
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
	Beacon b;

	//publish to "world_layout" topic
	b.position_pub = n.advertise<ugbots_ros::Position>("world_layout", 1000);
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		// create the msg to publish
		ugbots_ros::Position msg;
		msg.x = b.getX();
		msg.y = b.getY();

		// publish it
		b.position_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}

