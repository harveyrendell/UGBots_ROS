#include <speed.h>
#include <pose.h>
#include <subscriber_list.h>

class Unit

{
public:
	void publish()
	{
		//messages to stage
		node_cmdvel.linear.x = speed.linear_x;
		node_cmdvel.angular.z = speed.angular_z;
	        
		//publish the message
		sub_list.node_stage_pub.publish(node_cmdvel);
	}

	virtual void move() = 0;
	virtual void stop() = 0;
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
};
