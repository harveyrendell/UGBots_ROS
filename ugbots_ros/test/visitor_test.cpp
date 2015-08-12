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

void odom_callback(nav_msgs::Odometry msg)
{
	//Mock callback function
}

void laser_callback(sensor_msgs::LaserScan msg)
{
	//Mock callback function
}

TEST(UnitTest, testNodeInstantiation)
{
	
	EXPECT_EQ(node.speed.max_linear_x, 3.0);
	EXPECT_EQ(node.speed.linear_x, 3.0);
	EXPECT_EQ(node.speed.angular_z, 2.0);
}

TEST(TestSuite, testStartupState)
{
	EXPECT_EQ(node.state, Visitor::IDLE); 
}

/*
TEST(TestSuite, )
{

}

TEST(TestSuite, )
{

}

TEST(TestSuite, )
{

}
*/

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