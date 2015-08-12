// Bring in gtest
#include <gtest/gtest.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <stdlib.h>

#include <node_defs/carrier.h>

// Declare a test
TEST(UnitTest, testNodeInstantiation)
{
	ros::init(a, b, "CARRIER");
	ros::NodeHandle n;
	Carrier c(n);

	EXPECT_EQ(c.speed.max_linear_x, 3.0);
}


TEST(TestSuite, testCase2)
{
	EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	
	ros::init(a, b, "CARRIER");
	ros::NodeHandle n;
	Carrier c(n);

	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}