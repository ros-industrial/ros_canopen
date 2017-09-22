// Bring in my package's API, which is what I'm testing
#include <socketcan_interface/filter.h>


#include <socketcan_interface/string.h>

// Bring in gtest
#include <gtest/gtest.h>


TEST(FilterTest, simpleMask)
{
  const std::string msg1("123#");
  const std::string msg2("124#");

  can::FrameFilter::Ptr f1 = can::tofilter("123");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
}

TEST(FilterTest, maskTests)
{
  const std::string msg1("123#");
  const std::string msg2("124#");
  const std::string msg3("122#");

  can::FrameFilter::Ptr f1 = can::tofilter("123:123");
  can::FrameFilter::Ptr f2 = can::tofilter("123:ffe");
  can::FrameFilter::Ptr f3 = can::tofilter("123~123");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
  EXPECT_FALSE(f1->pass(can::toframe(msg3)));


  EXPECT_TRUE(f2->pass(can::toframe(msg1)));
  EXPECT_FALSE(f2->pass(can::toframe(msg2)));
  EXPECT_TRUE(f2->pass(can::toframe(msg3)));

  EXPECT_FALSE(f3->pass(can::toframe(msg1)));
  EXPECT_TRUE(f3->pass(can::toframe(msg2)));
  EXPECT_TRUE(f3->pass(can::toframe(msg3)));

}

TEST(FilterTest, rangeTest)
{
  const std::string msg1("120#");
  const std::string msg2("125#");
  const std::string msg3("130#");

  can::FrameFilter::Ptr f1 = can::tofilter("120-120");
  can::FrameFilter::Ptr f2 = can::tofilter("120_120");
  can::FrameFilter::Ptr f3 = can::tofilter("120-125");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
  EXPECT_FALSE(f1->pass(can::toframe(msg3)));

  EXPECT_FALSE(f2->pass(can::toframe(msg1)));
  EXPECT_TRUE(f2->pass(can::toframe(msg2)));
  EXPECT_TRUE(f2->pass(can::toframe(msg3)));

  EXPECT_TRUE(f3->pass(can::toframe(msg1)));
  EXPECT_TRUE(f3->pass(can::toframe(msg2)));
  EXPECT_FALSE(f3->pass(can::toframe(msg3)));

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
