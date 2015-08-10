
#include <unit.h>


class Node : public Unit

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

	virtual void odom_callback(nav_msgs::Odometry msg) = 0;
	virtual void laser_callback(sensor_msgs::LaserScan msg) = 0;
	virtual void move() = 0;
	virtual void stop() = 0;
	virtual void turnLeft() = 0;
	virtual void turnRight() = 0;
	virtual void collisionDetected() = 0;

	ros::NodeHandle n;
};
