// Bring in gtest
#include <gtest/gtest.h>

#include <sstream>
#include <stdlib.h>

//#include <node.h>
//#include <node_defs/carrier.h>

// Declare a test
TEST(TestSuite, testCase1)
{
	EXPECT_TRUE(true);
	/*ros::NodeHandle n;
	Carrier c(n);

	EXPECT_EQ(c.pose.px, 10);
	EXPECT_EQ(c.pose.py, 20);
	EXPECT_EQ(c.pose.theta = M_PI/2.0);
	EXPECT_EQ(c.speed.linear_x = 30.0);
	EXPECT_EQ(c.speed.angular_x = 20.0);
	EXPECT_EQ(c.speed.max_linear_x = 3.0);*/
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}