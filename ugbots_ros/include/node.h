#include <node_structs/speed.h>
#include <node_structs/pose.h>
#include <node_structs/subscriber_list.h>
#include <node_structs/orientation.h>

class Node
{
public:
	virtual void move() = 0;
	virtual void stop() = 0;
	virtual void collisionDetected() = 0;
	
	void publish()
	{
		//messages to stage
		node_cmdvel.linear.x = speed.linear_x;
		node_cmdvel.angular.z = speed.angular_z;
		//publish the message
		sub_list.node_stage_pub.publish(node_cmdvel);
	}

	void turn(double angle, double linear, double angular)
	{
			this->orientation.currently_turning = true;
		this->orientation.desired_angle = this->orientation.desired_angle + angle;
		this->speed.linear_x = linear;
		this->speed.angular_z = angular;
	}

	void doAngleCheck()
	{		
		//if -ve rads, change to +ve rads
		if(this->orientation.angle < 0)
		{
			this->orientation.angle = this->orientation.angle + 2.000000 * M_PI;
		}

		if(this->orientation.desired_angle < 0)
		{
			this->orientation.desired_angle = this->orientation.desired_angle + 2.000000 * M_PI;
		}
		//if the desired angle is > 2pi, changed the desired angle to pi/2 
		if(this->orientation.desired_angle > (2.000000 * M_PI))
		{
			this->orientation.desired_angle = M_PI / 2.000000;
		}
		//if the current angle is 2pi or more, translate the angle to 0< x <2pi 
		if(this->orientation.angle > 2.000000 * M_PI)
		{
			this->orientation.angle = this->orientation.angle - 2.000000 * M_PI;	
		}
	}

	//
	void checkTurningStatus()
	{
		//Implement individually.
		//Change 2-3 to which ever suits your node
		//parse in ur desired linear speed to stopTurn()

		/*
		if(this->orientation.currently_turning == true)
		{
			if((this->orientation.angle + (M_PI / (speed.angular_z * 2) ) ) == this->orientation.desired_angle)
			{
				stopTurn();
			}
			return;
		}
		*/
	}


	//calculates current orientation using atan2
	void calculateOrientation()
	{	
		this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);
	}



	//Pose of the unit
	Pose pose;

	//Velocity of the unit
	Speed speed;

	//NodeHandle for the node
	//ros::NodeHandle n;

	//Velocity of the robot
	geometry_msgs::Twist node_cmdvel;

	//Publishers and Subscribers for node
	SubscriberList sub_list;

	//Orientation of the unit
	Orientation orientation;
};
