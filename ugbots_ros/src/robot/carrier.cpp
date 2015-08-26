#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>
#include <node_defs/carrier.h>

Carrier::Carrier()
{
	//setting base attribute defaults
	pose.theta = M_PI/2.0;
	pose.px = 10;
	pose.py = 20;
	speed.linear_x = 0.0;
	speed.linear_y = 0.0;

	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;

	moving = false;
	undergoing_task = false;

	idle_status_sent = false;
	station_set = false;

	temprad = 0.0;
}

Carrier::Carrier(ros::NodeHandle &n)
{
	this->nh = n;

	//setting base attribute defaults
	speed.linear_x = 0.0;
	speed.linear_y = 0.0;
	speed.max_linear_x = 3.0;
	speed.angular_z = 0.0;
	state = IDLE;
	orientation.currently_turning = false;

	x_completed = false;
	x_started = false;
	y_completed = false;
	x_started = false;


	idle_status_sent = false;
	station_set = false;
	std::string ns = n.getNamespace();
	ns.erase(ns.begin());
	robotDetails.ns = ns;

	core_alert = n.advertise<ugbots_ros::robot_details>("/idle_carriers",1000);
	sub_bin = n.subscribe<ugbots_ros::robot_details>("bin", 1000, &Carrier::bin_loc_callback, this);

	sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, &Carrier::odom_callback, this);
	sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,&Carrier::laser_callback, this);
	carrier_alert = n.subscribe<ugbots_ros::bin_status>("/alert",1000,&Carrier::bin_callback,this);
	carrier_alert_pub = n.advertise<ugbots_ros::bin_status>("/alert",1000);
}

void Carrier::bin_callback(ugbots_ros::bin_status msg)
{
	localBinStatus = msg;
}


char const* Carrier::enum_to_string(State t){
    switch(t){
        case IDLE:
            return "IDLE";
        case TRAVELLING:
            return "TRAVELLING";
        case CARRYING:
            return "CARRYING";
        case AVOIDING:
            return "AVOIDING";
        case STOPPED:
            return "STOPPED";     
        default:
            return "INVALID ENUM";
    }
 }


void Carrier::odom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	pose.px = msg.pose.pose.position.x;
	pose.py = msg.pose.pose.position.y;
	orientation.rotx = msg.pose.pose.orientation.x;
	orientation.roty = msg.pose.pose.orientation.y;
	orientation.rotz = msg.pose.pose.orientation.z;
	orientation.rotw = msg.pose.pose.orientation.w;

	if (!station_set) {
		station_x = pose.px;
		station_y = pose.py;
	}

	calculateOrientation();
	if (state == IDLE && !idle_status_sent) 
	{
		robotDetails.x = pose.px;
		robotDetails.y = pose.py;
		core_alert.publish(robotDetails);
		idle_status_sent = true;
	}

	//orientation.angle = atan2(2*(orientation.roty*orientation.rotx+orientation.rotw*orientation.rotz),
	//orientation.rotw*orientation.rotw+orientation.rotx*orientation.rotx-orientation.roty*
	//orientation.roty-orientation.rotz*orientation.rotz);
	ROS_INFO("/position/x/%f", this->pose.px);
	ROS_INFO("/position/y/%f", this->pose.py);
	//ROS_INFO("/orientation/angle/%f", this->orientation.angle);
	//ROS_INFO("/speed/x/%f", msg.twist.twist.linear.x);
	//ROS_INFO("/speed/y/%f", msg.twist.twist.linear.y);
	ROS_INFO("/status/%s/./", enum_to_string(state));

	if(localBinStatus.bin_stat == "FULL")
	{
		geometry_msgs::Point location_point;
		location_point.x = localBinStatus.bin_x;
		location_point.y = localBinStatus.bin_y;
		action_queue.push(location_point);
	}

	if (state == IDLE) {
		begin_action(0);
	} else if (state == TRAVELLING) {
		begin_action(3);
	} else if (state == CARRYING) {
		if (!picker_bin_msg_sent) {
			std::string topic = associated_picker + "/bin_emptied";
			picker_alert = nh.advertise<std_msgs::String>(topic,1000,true);
			picker_alert.publish(topic);
			picker_bin_msg_sent = true;
		}
		begin_action(1.5);
	} else if (state == STOPPED) {
		speed.linear_x = 0;
	}

	doAngleCheck();
	checkTurningStatus();
	publish();
}


void Carrier::laser_callback(sensor_msgs::LaserScan msg)
{
	bool detected = false;
	for (int i = 55; i < 126; i++) {
		if (msg.ranges[i] < 1.29035) {
			detected = true;
		}
	}
	if (detected) {
		state = STOPPED;
	} else {
		begin_action(0);
	}

	/*
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

			/*
				pointtemp.x = this->pose.px; 
				pointtemp.y = this->pose.py + 1.1;

				temp_queue.push(pointtemp);

				pointtemp.x = this->pose.px - 4.0; 
				pointtemp.y = this->pose.py + 1.1;

				temp_queue.push(pointtemp);

				pointtemp.x = this->pose.px - 4.0; 
				pointtemp.y = this->pose.py;

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
	}**/
}

void Carrier::bin_loc_callback(ugbots_ros::robot_details bin)
{
	//picker bin has not been emptied, hence false
	picker_bin_msg_sent = false;
	associated_picker = bin.ns;

	//set first point of action, which is the pivot point
	geometry_msgs::Point pivot;
	pivot.x = bin.x;
	pivot.y = -40;
	action_queue.push(pivot);
	state_queue.push(1);
	//set goal point of picking up bin
	geometry_msgs::Point bin_location;
	bin_location.x = bin.x;
	bin_location.y = bin.y;
	action_queue.push(bin_location);
	state_queue.push(1);
	//set the carry course for the carrier back to its station
	geometry_msgs::Point carry_point;
	carry_point.x = station_x;
	carry_point.y = -42;
	action_queue.push(carry_point);
	state_queue.push(2);
	carry_point.y = station_y;
	action_queue.push(carry_point);
	state_queue.push(2);
}

void Carrier::set_status(int status){
	for(int i = 0; i < arraysize(state_array); i++)
	{
		if(i == status)
		{
			state = state_array[i];
			if (state == IDLE) {
				idle_status_sent = false;
				picker_bin_msg_sent = false; 
			}
		}
	}
}

void Carrier::stop()
{
	speed.linear_x = 0.0;
	speed.linear_y = 0.0;
	speed.angular_z = 0.0;
}



int main(int argc, char **argv)
{	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "CARRIER");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//Creating the CarrierBot instance
Carrier node(n);

node.binStatus.bin_stat = "EMPTY";

//Setting the loop rate
ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

while (ros::ok())
{
	//node.publish();	

	node.carrier_alert_pub.publish(node.binStatus);

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}
return 0;

}
