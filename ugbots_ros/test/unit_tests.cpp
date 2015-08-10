// Bring in gtest
#include <gtest/gtest.h>
#include <node.h>
#include ""

// Declare a test
TEST(TestSuite, testCase1)
{

	EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}