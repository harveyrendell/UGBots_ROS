#include <node_structs/speed.h>
#include <node_structs/pose.h>
#include <node_structs/subscriber_list.h>
#include <node_structs/orientation.h>
#include <queue> 

class Node
{
public:
	virtual void move() = 0;
	virtual void stop() = 0;
	virtual void collisionDetected() = 0;
	//virtual void set_status(int i);

	void publish()
	{
		//messages to stage
		node_cmdvel.linear.x = speed.linear_x;
		node_cmdvel.linear.y = speed.linear_y;
		node_cmdvel.angular.z = speed.angular_z;
		//publish the message
		sub_list.node_stage_pub.publish(node_cmdvel);
	}

	/*void turn(double angle, double linear, double angular)
	{
		this->orientation.currently_turning = true;
		this->orientation.desired_angle = this->orientation.desired_angle + angle;
		this->speed.linear_x = linear;
		this->speed.angular_z = angular;
	}*/

	void turn(double angle, double linear, double angular)
	{
		this->orientation.currently_turning = true;
		this->orientation.desired_angle = this->orientation.desired_angle + angle;
		double angle_difference = fabs(this->orientation.desired_angle - this->orientation.angle);
		
		/*if(angle_difference < M_PI/10)
		{
			angular = M_PI/1800;
		}**/

		doAngleCheck();
		this->speed.angular_z = angular;
		this->speed.linear_x = linear;
	}
	bool begin_action_shortest_path(double speed)
	{
		if(action_queue.empty())
		{
			//set_status(1);
			return true;
		}
		geometry_msgs::Point end_point = action_queue.front();
		if(doubleComparator(end_point.x, pose.px) && doubleComparator(end_point.y, pose.py))
		{
			action_queue.pop();
			stop();
			return true;
		}
		double distance = sqrt(pow(end_point.x - pose.px, 2) + pow(end_point.y - pose.py, 2));
		double angle = atan2((end_point.y - pose.py),(end_point.x - pose.px));

		turn(angle - this->orientation.desired_angle , 0.0, M_PI/2);
		ROS_INFO("DESIRED ANGLE: %f", this->orientation.desired_angle);
		ROS_INFO("ANGLE: %f", this->orientation.angle);
		checkTurningStatus();
		if(!orientation.currently_turning)
		{
			this->speed.linear_x = speed;
			if (fabs(distance) < 0.5)
			{
				this->speed.linear_x = distance;
			}
		}
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
		while(this->orientation.desired_angle >= (2.000000 * M_PI))
		{
			this->orientation.desired_angle = this->orientation.desired_angle - 2.000000 * M_PI;
		}
		//if the current angle is 2pi or more, translate the angle to 0< x <2pi 
		while(this->orientation.angle >= 2.000000 * M_PI)
		{
			this->orientation.angle = this->orientation.angle - 2.000000 * M_PI;	
		}
	}

	//
	void checkTurningStatus()
	{
		if(this->orientation.currently_turning == true)
		{	
			if(doubleComparator(orientation.angle, orientation.desired_angle))
			{
				this->orientation.currently_turning = false;
				this->speed.linear_x = 3.0;
				this->speed.angular_z = 0.0; 
			}
		return;
		}
	}

	//calculates current orientation using atan2
	void calculateOrientation()
	{	
		this->orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*orientation.roty-orientation.rotz*orientation.rotz);
	}


	bool move_x(double distance, double speed) {
		double distance_x = distance - pose.px;
		if (fabs(distance_x) < M_PI/18000) {
			ROS_INFO("X DONE");
			stop();
			return true;
		}
		if (distance_x < 0.0)
		{
			turn(M_PI - this->orientation.desired_angle , 0.0, M_PI/2);
			checkTurningStatus();
		} 
		else 
		{
			turn(-1.0 * this->orientation.desired_angle , 0.0, M_PI/2);
			checkTurningStatus();
		}
		if(!orientation.currently_turning)
		{
			this->speed.linear_x = speed;
			if (fabs(distance_x) < 0.5)
			{
				//ROS_INFO("slow down x");
				this->speed.linear_x = fabs(distance_x);
			}
		}
		return false;
	}

	bool move_y(double distance, double speed) {
		double distance_y = distance - pose.py;
		if (fabs(distance_y) < M_PI/18000) {
			ROS_INFO("Y DONE");
			stop();
			return true;
		}
		if (distance_y < 0.0)
		{
			turn(-1.0 * M_PI/2.0 - this->orientation.desired_angle , 0.0, M_PI/2);
			checkTurningStatus();
		}
		else
		{
			turn(M_PI/2.0 - this->orientation.desired_angle , 0.0, M_PI/2);
			checkTurningStatus();	
		}
		if(!orientation.currently_turning)
		{
			this->speed.linear_x = speed;
			if (fabs(distance_y) < 0.5)
			{
				//ROS_INFO("slow down x");
				this->speed.linear_x = fabs(distance_y);
			}
		}
		return false;
	}

	
	bool begin_action(double speed)
	{

		if (action_queue.empty())
		{
			//set_status(1);
			return true;
		}
		geometry_msgs::Point end_point = action_queue.front();
		if(doubleComparator(end_point.x, pose.px) && doubleComparator(end_point.y, pose.py))
		{
			ROS_INFO("xdest: %f", end_point.x);
			ROS_INFO("ydest: %f", end_point.y);
			action_queue.pop();
			stop();
			return true;
		}

		if(move_x(end_point.x, speed))
		{
			if(move_y(end_point.y, speed))
			{
				//set_status(2);
			}

		}
	}	

	bool doubleComparator(double a, double b)
	{
	    return fabs(a - b) < M_PI/18000;
	}



	//Pose of the unit
	Pose pose;

	//Velocity of the unit
	Speed speed;

	//Queue of the Actions
	std::queue<geometry_msgs::Point> action_queue;

	//Beacon points
	std::list<geometry_msgs::Point> beacon_points;

	//NodeHandle for the node
	//ros::NodeHandle n;

	//Velocity of the robot
	geometry_msgs::Twist node_cmdvel;

	//Publishers and Subscribers for node
	SubscriberList sub_list;

	//Orientation of the unit
	Orientation orientation;


};
