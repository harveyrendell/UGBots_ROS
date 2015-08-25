#include <node_structs/speed.h>
#include <node_structs/pose.h>
#include <node_structs/subscriber_list.h>
#include <node_structs/orientation.h>
#include <queue>

class Node
{
public:
	virtual void stop() = 0;
	virtual void set_status(int status){}

	template <typename T,unsigned S>
	inline unsigned arraysize(const T (&v)[S]) { return S; }

	void publish()
	{
		//messages to stage
		node_cmdvel.linear.x = speed.linear_x;
		node_cmdvel.linear.y = speed.linear_y;
		node_cmdvel.angular.z = speed.angular_z;
		//publish the message
		sub_list.node_stage_pub.publish(node_cmdvel);
	}

	void turn(double angle, double linear, double angular)
	{
		this->orientation.currently_turning = true;
		this->orientation.desired_angle = this->orientation.desired_angle + angle;
		double angle_difference = fabs(this->orientation.desired_angle - this->orientation.angle);

		angular = deceleration(angle_difference, M_PI/10, M_PI/18000000);
		doAngleCheck();

		if(this->orientation.desired_angle > this->orientation.angle)
		{
			if(this->orientation.desired_angle - this->orientation.angle > M_PI)
			{
				//if desired angle - orientation is greater than pi radians set angular to clockwise
				if (angular > 0)
					angular = angular * -1.0;
			}
			else
			{
				if (angular < 0)
					angular = angular * -1.0; 
			}
		}
		else
		{
			if(this->orientation.angle - this->orientation.desired_angle > M_PI)
			{
				//if desired angle - orientation is greater than pi radians set angular to clockwise
				if (angular < 0)
					angular = angular * -1.0; 
			}
			else
			{
				if (angular > 0)
					angular = angular * -1.0;	
			}	
		}

		this->speed.angular_z = angular;
		this->speed.linear_x = linear;
	}
	double deceleration(double distance, double tolerance, double max_tolerance)
	{
		if(distance < tolerance)
		{
			if (tolerance == M_PI/10)
			{
				return deceleration(distance, M_PI/180, max_tolerance);
			}
			if (tolerance < max_tolerance)
			{
				return max_tolerance;
			}
			return deceleration(distance, tolerance/10, max_tolerance);
		}
		return tolerance * 10;
	}

	bool begin_action_shortest_path(double speed)
	{
		if(action_queue.empty())
		{
			set_status(0);
			return true;
		}
		geometry_msgs::Point end_point = action_queue.front();
		set_status(state_queue.front());
		if(doubleComparator(end_point.x, pose.px) && doubleComparator(end_point.y, pose.py))
		{
			action_queue.pop();
			state_queue.pop();
			stop();
			return true;
		}
		double distance = sqrt(pow(end_point.x - pose.px, 2) + pow(end_point.y - pose.py, 2));
		double angle = atan2((end_point.y - pose.py),(end_point.x - pose.px));

		turn(angle - this->orientation.desired_angle , 0.0, M_PI/2);
		checkTurningStatus();
		if(!orientation.currently_turning)
		{
			set_status(1);
			speed = deceleration(fabs(distance), 1, 0.005);
			this->speed.linear_x = speed;
			if (fabs(distance) < 0.5)
			{
				this->speed.linear_x = distance;
			}
		}
		return false;
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
			//ROS_INFO("desired angle: %f", orientation.desired_angle);
			//ROS_INFO("angle: %f", orientation.angle);
			if(doubleAngleComparator(orientation.angle, orientation.desired_angle))
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
		if (fabs(distance_x) < 0.0005) {
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
			//if(fabs(distance_x) < 0.1)
			speed = deceleration(fabs(distance_x), 1, 0.01);
			this->speed.linear_x = speed;
		}
		return false;
	}

	bool move_y(double distance, double speed) {
		double distance_y = distance - pose.py;
		if (fabs(distance_y) < 0.0005) {
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
			return true;
		}
		geometry_msgs::Point end_point = action_queue.front();
		set_status(state_queue.front());

		if(doubleComparator(end_point.x, pose.px) && doubleComparator(end_point.y, pose.py))
		{
			//ROS_INFO("xdest: %f", end_point.x);
			//ROS_INFO("ydest: %f", end_point.y);
			action_queue.pop();
			state_queue.pop();
			if (action_queue.empty()) {
				set_status(0);
			}
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
		return false;
	}	

	bool doubleAngleComparator(double a, double b)
	{
	    return fabs(a - b) < M_PI/3600000;
	}
	bool doubleComparator(double a, double b)
	{
	    return fabs(a - b) < 0.0005;
	}	




	//Pose of the unit
	Pose pose;

	//Velocity of the unit
	Speed speed;

	//Queue of the Actions
	std::queue<geometry_msgs::Point> action_queue;
	std::queue<int> state_queue;
	//NodeHandle for the node
	//ros::NodeHandle n;

	//Velocity of the robot
	geometry_msgs::Twist node_cmdvel;

	//Publishers and Subscribers for node
	SubscriberList sub_list;

	//Orientation of the unit
	Orientation orientation;


};
