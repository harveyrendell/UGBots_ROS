// Bring in gtest
#include <gtest/gtest.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>

#include <node_defs/visitor.h>

Visitor node;
bool odom_cb = false;
bool laser_cb = false;

void setup()
{
	odom_cb = false;
	laser_cb = false;
	node = Visitor();
}

void odom_callback(nav_msgs::Odometry msg)
{
	//Mock callback function
	odom_cb = true;
}

void laser_callback(sensor_msgs::LaserScan msg)
{
	//Mock callback function
	laser_cb = true;
}

TEST(UnitTest, testNodeInitialisedSpeed)
{
	setup();

	EXPECT_EQ(node.speed.linear_x, 0.0);
	EXPECT_EQ(node.speed.angular_z, 0.0);
}

TEST(UnitTest, testNodeTopSpeed)
{
	setup();

	EXPECT_EQ(node.speed.max_linear_x, 3.0);
}

TEST(UnitTest, testStartupState)
{
	setup();

	EXPECT_EQ(node.state, Visitor::IDLE); 
}

//Initialises a route and tests that it is added to the action_queue
TEST(AcceptanceTest, testActionQueueInit)
{
	setup();

	node.init_route();
	EXPECT_TRUE(!node.action_queue.empty());
}

TEST(AcceptanceTest, testActionQueue)
{
	//node.action_queue.
	//EXPECT_EQ(node.state, Visitor::IDLE); 
}

TEST(AcceptanceTest, testTurnStop)
{
	setup();

	node.orientation.currently_turning = true;
	node.orientation.desired_angle = node.orientation.angle;

	node.checkTurningStatus();

	EXPECT_FALSE(node.orientation.currently_turning);
	//EXPECT_TRUE(node.orientation.angle, node.orientation.desired_angle)
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	//Create a node to test with
	ros::init(argc, argv, "VISITOR");
	ros::NodeHandle n;
	
	node.sub_list.node_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	node.sub_list.sub_odom = n.subscribe<nav_msgs::Odometry>("odom",1000, odom_callback);
	node.sub_list.sub_laser = n.subscribe<sensor_msgs::LaserScan>("base_scan",1000,laser_callback);

	//Run the test suite
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}