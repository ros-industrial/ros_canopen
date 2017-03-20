#include <gtest/gtest.h>
#include <canopen_motor_node/unit_converter.h>
#include <canopen_motor_node/handle_layer.h>
#include <boost/bind.hpp>

using namespace canopen;

double * mapVariable(const std::string &, double *p) {
    return p;
}

TEST(TestMuparser, CheckNorm){
    double input = 0;
    UnitConverter uc("norm(in,-1000,1000)", boost::bind(HandleLayer::assignVariable, _1, &input, "in"));
    input = 0; EXPECT_EQ(0, uc.evaluate());
    input = 10; EXPECT_EQ(10, uc.evaluate());
    input = -10; EXPECT_EQ(-10, uc.evaluate());
    input = 1000; EXPECT_EQ(-1000, uc.evaluate());
    input = 1001; EXPECT_EQ(-999, uc.evaluate());
    input = 2000; EXPECT_EQ(0, uc.evaluate());
    input = 2001; EXPECT_EQ(1, uc.evaluate());
    input = -1000; EXPECT_EQ(-1000, uc.evaluate());
    input = 999; EXPECT_EQ(999, uc.evaluate());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
