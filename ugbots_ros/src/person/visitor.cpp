#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/visitor.h>

Visitor::Visitor()
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;
}

Visitor::Visitor(ros::NodeHandle &n)
{
	//this->n = n;

	//setting base attribute defaults
	this->pose.theta = M_PI/2.0;
	this->pose.px = -40;
	this->pose.py = -44;
	this->speed.linear_x = 2.0;
	this->speed.max_linear_x = 3.0;
	this->speed.angular_z = 0.0;
	this->state = IDLE;

	this->orientation.previous_right_distance = 0;
	this->orientation.previous_left_distance = 0;
	this->orientation.previous_front_distance = 0;
	this->orientation.angle = 0;
	this->orientation.desired_angle = 0;

	this->orientation.currently_turning = false;
	this->orientation.currently_turning_static = false;

	this->rightTurnInit = false;
	this->leftTurnInit = false;
	this->moveToEnabled = true;
	this->queueDuplicateCheckAngle = 0.0;
	this->queueDuplicate = true;

	this->sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	this->sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Visitor::odom_callback, this);
	this->sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Visitor::laser_callback, this);
	
	this->sub_row = n.subscribe<ugbots_ros::Position>("/row_loc",1000,&Visitor::core_callback, this);

	init_route();

}

void Visitor::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	this->pose.px = msg.pose.pose.position.x;
	this->pose.py = msg.pose.pose.position.y;
	this->orientation.rotx = msg.pose.pose.orientation.x;
	this->orientation.roty = msg.pose.pose.orientation.y;
	this->orientation.rotz = msg.pose.pose.orientation.z;
	this->orientation.rotw = msg.pose.pose.orientation.w;

	calculateOrientation();

	if(this->moveToEnabled == true)
	{
		begin_action_shortest_path(2.0);
	}

	doAngleCheck();		

	checkTurningStatus();

	publish();



	//ROS_INFO("LINEAR SPEED: %f", this->speed.linear_x);
	//ROS_INFO("ANGULAR SPEED: %f", this->speed.angular_z);


	//ROS_INFO("ANGLE: %f",this->orientation.angle);
	//ROS_INFO("DESIRED ANGLE: %f", this->orientation.desired_angle);


	//checkStaticTurningStatus();

	//ROS_INFO("/position/x/%f",action_queue.front().x);
	//ROS_INFO("/position/y/%f",action_queue.front().y);
	ROS_INFO("/status/TEMP/./");
	/*
	//ROS_INFO("ANGLE: %f",this->orientation.angle);
	//ROS_INFO("DESIRED ANGLE: %f", this->orientation.desired_angle);
	*/
}


void Visitor::laser_callback(sensor_msgs::LaserScan msg)
{
	
	if(fabs(this->queueDuplicateCheckAngle - this->orientation.angle) >= (M_PI/2.000000))
	{
		this->queueDuplicate = true;
		this->queueDuplicateCheckAngle = 0;
	}
	

	if(msg.ranges[90] < 2.0)
	{
		if(this->queueDuplicate == true)
		{
			this->queueDuplicateCheckAngle = this->orientation.angle;

			std::queue<geometry_msgs::Point> temp_queue;

			geometry_msgs::Point pointtemp;

			
			pointtemp.x = this->pose.px + 2 * cos(this->orientation.angle - (M_PI/2.0));
			pointtemp.y = this->pose.py + 2 * sin(this->orientation.angle - (M_PI/2.0));
			temp_queue.push(pointtemp);

			pointtemp.x = pointtemp.x + 4 * cos(this->orientation.angle);
			pointtemp.y = pointtemp.y + 4 * sin(this->orientation.angle);
			temp_queue.push(pointtemp);

			pointtemp.x = pointtemp.x + 2 * cos(this->orientation.angle + (M_PI/2.0));
			pointtemp.y = pointtemp.y + 2 * sin(this->orientation.angle + (M_PI/2.0));
			temp_queue.push(pointtemp);

			while(!action_queue.empty())
			{
				temp_queue.push(action_queue.front());
				action_queue.pop();
			}

			while(!temp_queue.empty())
			{
				action_queue.push(temp_queue.front());
				temp_queue.pop();
			}

			this->queueDuplicate = false;
		}
	}
}

void Visitor::core_callback(ugbots_ros::Position msg)
{
	ROS_INFO("/position/x/%f",msg.x);
	ROS_INFO("/position/y/%f",msg.y);
}

	/*if(msg.ranges[90] < 2.0 && msg.ranges[0] < 8.0)
	{
		if(this->rightTurnInit == false)
		{
			if(this->orientation.currently_turning == false)
			{
				ROS_INFO("RIGHT TURN!");
				this->moveToEnabled = false;
				turn(-(M_PI/2.000000) , 0.5, -5.0);
				this->rightTurnInit = true;
			}
		}
	}

	if(msg.ranges[90] < 3.0 && this->rightTurnInit == true)
	{
		if(this->leftTurnInit == false)
		{
			if(this->orientation.currently_turning == false)
			{
				ROS_INFO("LEFT TURN!");
				this->moveToEnabled = false;
				turn((M_PI/2.000000) , 0.5, 5.0);
				this->leftTurnInit = true;
			}
		}
	}*/

void Visitor::checkTurningStatus()
{
		//Implement individually.
		//Change 2-3 to which ever suits your node
		//parse in ur desired linear speed to stopTurn()
		
	if(this->orientation.currently_turning == true)
	{
		if(doubleComparator(this->orientation.angle , this->orientation.desired_angle))
		{
			this->orientation.currently_turning = false;
			this->speed.linear_x = 2.0;
			this->speed.angular_z = 0.0;
		}
	}
}

			/*if(this->leftTurnInit == true && this->rightTurnInit == true)
			{
				ROS_INFO("STOP TURN!");
				this->leftTurnInit = false;
				this->rightTurnInit = false;

				std::queue<geometry_msgs::Point> temp_queue;

				geometry_msgs::Point pointtemp;
				pointtemp.x = this->pose.px; 
				pointtemp.y = this->pose.py + 5.0;

				temp_queue.push(pointtemp);

				while(!action_queue.empty())
				{
					temp_queue.push(action_queue.front());
					action_queue.pop();
				}

				while(!temp_queue.empty())
				{
					action_queue.push(temp_queue.front());
					temp_queue.pop();
				}

				this->moveToEnabled = true;
			}
		}
	}
		
}*/

void Visitor::init_route()
{
	geometry_msgs::Point point;
		point.x = 10.5; 
		point.y = -39.0;

	action_queue.push(point);
	
		point.x = 10.5;
		point.y = 39.0;

	action_queue.push(point);

		point.x = 3.5;
		point.y = 39.0;

	action_queue.push(point);

		point.x = 3.5;
		point.y = -39.0;

	action_queue.push(point);

		point.x = -3.5;
		point.y = -39.0;

	action_queue.push(point);

		point.x = -3.5;
		point.y = 39.0;

	action_queue.push(point);

		point.x = -10.5;
		point.y = 39.0;

	action_queue.push(point);

		point.x = -10.5;
		point.y = -39.0;

	action_queue.push(point);

		point.x = 52.0;
		point.y = -48.5;

	action_queue.push(point);
}

void Visitor::move(){}
void Visitor::stop(){}
void Visitor::turnLeft(){}
void Visitor::turnRight(){}
void Visitor::collisionDetected(){}
char const* Visitor::enum_to_string(State t){ return ""; }

int main(int argc, char **argv)
{	
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "VISITOR");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Visitor node(n);

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

while (ros::ok())
{
	
	//node.publish();

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
