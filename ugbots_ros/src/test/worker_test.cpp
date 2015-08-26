// Bring in gtest
#include <gtest/gtest.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>

#include <node_defs/worker.h>

static Worker node;

void setup()
{
	// create a new instance of visitor
	node = Worker();
}

//######################### UNIT TESTS #########################

TEST(UnitTest, testNodeInitialisedSpeed)
{
	setup();

	EXPECT_EQ(node.speed.linear_x, 0.0);
	EXPECT_EQ(node.speed.angular_z, 0.0);
}

TEST(UnitTest, testNodeTopSpeed)
{
	EXPECT_EQ(node.speed.max_linear_x, 3.0);
}

TEST(UnitTest, testStartupState)
{
	setup();

	EXPECT_EQ(node.state, Worker::IDLE); 
}

//###################### ACCEPTANCE TESTS ######################

TEST(AcceptanceTest, testLetVisitorIn)
{
	setup();

	node.letInNextVisitor();
	EXPECT_TRUE(!node.action_queue.empty()); 
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	//Create a node to test with
	ros::init(argc, argv, "WORKER");
	ros::NodeHandle n;

	//Run the test suite
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}