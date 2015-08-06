#include "Speed.h"
#include "Pose.h"
#include "SubscriberList.h"
class Unit

{
public:
	void publish()
	{
	
	}

	virtual void moveTo(int x, int y){}
	virtual void stop (){}
	virtual void collisionDetected(){}

	//pose of the unit
	Pose pose;

	//velocity of the unit
	Speed speed;

	//NodeHandle for the node
	ros::NodeHandle n;

	//Velocity of the robot
	geometry_msgs::Twist node_cmdvel;

	SubscriberList sub_list;
}
