#include <joint_limits_controller/joint_limiter.h>
#include <urdf/model.h>
#include <ros/ros.h>

#include <gtest/gtest.h>
bool parseURDF(ros::NodeHandle nh, urdf::Model &urdf, bool &has_urdf){ // TODO: migrate this function to somewhere else
   std::string has_robot_description_fqn;
   bool has_robot_description = true;

   if(nh.searchParam("has_robot_description", has_robot_description_fqn)){
        ros::param::get(has_robot_description_fqn, has_robot_description);
    }
    if(has_robot_description){
        if(!urdf.initParam("robot_description")) return false;
        has_urdf = true;
    }else{
        has_urdf = false;
    }
    return true;
}

TEST(TestURDF, testNoURDF1)
{
    ros::NodeHandle nh("test1");
    urdf::Model urdf;
    bool has_urdf;

    ASSERT_TRUE(parseURDF(nh, urdf, has_urdf));
    ASSERT_FALSE(has_urdf);
}

TEST(TestURDF, testNoURDF2)
{
    ros::NodeHandle nh("test1/test2");
    urdf::Model urdf;
    bool has_urdf;

    ASSERT_TRUE(parseURDF(nh, urdf, has_urdf));
    ASSERT_FALSE(has_urdf);
}

TEST(TestURDF, testURDF3)
{
    ros::NodeHandle nh("test3");
    urdf::Model urdf;
    bool has_urdf;

    ASSERT_TRUE(parseURDF(nh, urdf, has_urdf));
    ASSERT_TRUE(has_urdf);
}


TEST(TestURDF, testNoURDF4)
{
    ros::NodeHandle nh("test3/test4");
    urdf::Model urdf;
    bool has_urdf;

    ASSERT_TRUE(parseURDF(nh, urdf, has_urdf));
    ASSERT_FALSE(has_urdf);
}

TEST(TestURDF, testURDF)
{
    ros::NodeHandle nh;
    urdf::Model urdf;
    bool has_urdf;

    ASSERT_TRUE(parseURDF(nh, urdf, has_urdf));
    ASSERT_TRUE(has_urdf);

    boost::shared_ptr<const urdf::Joint> joint1 = urdf.getJoint("joint1"), joint2 = urdf.getJoint("joint2");
    ASSERT_TRUE(joint1);
    ASSERT_TRUE(joint2);

    JointLimiter::Limits limits1(joint1);
    EXPECT_TRUE(limits1.hasPositionLimits());
    EXPECT_TRUE(limits1.hasVelocityLimits());
    EXPECT_FALSE(limits1.hasAccelerationLimits());
    EXPECT_FALSE(limits1.hasJerkLimits());
    EXPECT_TRUE(limits1.hasEffortLimits());
    EXPECT_TRUE(limits1.hasSoftLimits());

    EXPECT_TRUE(limits1.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_TRUE(limits1.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits1.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits1.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_TRUE(limits1.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_TRUE(limits1.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);

    EXPECT_EQ(-111, limits1.joint_limits.min_position);
    EXPECT_EQ(+111, limits1.joint_limits.max_position);
    EXPECT_EQ(11, limits1.joint_limits.max_velocity);
    EXPECT_EQ(1111, limits1.joint_limits.max_effort);

    EXPECT_EQ(-110, limits1.soft_limits.min_position);
    EXPECT_EQ(+110, limits1.soft_limits.max_position);
    EXPECT_EQ(10, limits1.soft_limits.k_position);
    EXPECT_EQ(12, limits1.soft_limits.k_velocity);

    JointLimiter::Limits limits2(joint2);

    EXPECT_TRUE(limits2.hasPositionLimits());
    EXPECT_TRUE(limits2.hasVelocityLimits());
    EXPECT_FALSE(limits2.hasAccelerationLimits());
    EXPECT_FALSE(limits2.hasJerkLimits());
    EXPECT_TRUE(limits2.hasEffortLimits());
    EXPECT_FALSE(limits2.hasSoftLimits());

    EXPECT_TRUE(limits2.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_TRUE(limits2.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits2.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits2.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_TRUE(limits1.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_FALSE(limits2.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);

    EXPECT_EQ(-222, limits2.joint_limits.min_position);
    EXPECT_EQ(+222, limits2.joint_limits.max_position);
    EXPECT_EQ(22, limits2.joint_limits.max_velocity);
    EXPECT_EQ(2222, limits2.joint_limits.max_effort);
}
void testPos(const JointLimiter::Limits &limits) {
    EXPECT_TRUE(limits.hasPositionLimits());
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_EQ(-333, limits.joint_limits.min_position);
    EXPECT_EQ(+333, limits.joint_limits.max_position);
}
void testVel(const JointLimiter::Limits &limits) {
    EXPECT_TRUE(limits.hasVelocityLimits());
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_EQ(+33, limits.joint_limits.max_velocity);
}
void testAcc(const JointLimiter::Limits &limits) {
    EXPECT_TRUE(limits.hasAccelerationLimits());
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_EQ(+3333, limits.joint_limits.max_acceleration);
}
void testJerk(const JointLimiter::Limits &limits) {
    EXPECT_TRUE(limits.hasJerkLimits());
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_EQ(+33333, limits.joint_limits.max_jerk);
}
void testEffort(const JointLimiter::Limits &limits) {
    EXPECT_TRUE(limits.hasEffortLimits());
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_EQ(+333333, limits.joint_limits.max_effort);
}
void testSoft(const JointLimiter::Limits &limits) {
    EXPECT_TRUE(limits.hasSoftLimits());
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
    EXPECT_EQ(-3, limits.soft_limits.min_position);
    EXPECT_EQ(+3, limits.soft_limits.max_position);
    EXPECT_EQ(4, limits.soft_limits.k_position);
    EXPECT_EQ(5, limits.soft_limits.k_velocity);
}

TEST(TestYAML, testPosYAML)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("only_pos"), true);
    testPos(limits);

    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasEffortLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}

TEST(TestYAML, testVelYAML)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("only_vel"), true);
    testVel(limits);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasEffortLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}

TEST(TestYAML, testAccYAML)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("only_acc"), true);
    testAcc(limits);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasEffortLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}

TEST(TestYAML, testJerkYAML)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("only_jerk"), true);
    testJerk(limits);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasEffortLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}
TEST(TestYAML, testEffortYAML)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("only_effort"), true);
    testEffort(limits);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}
TEST(TestYAML, testSoftYAML)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("only_soft"), true);
    testSoft(limits);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasEffortLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
}

TEST(TestYAML, testNoSoftYAML)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("only_soft"), false);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasEffortLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}

TEST(TestYAML, testNoLimits)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("no_limits"), true);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasEffortLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_TRUE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}

TEST(TestYAML, testEmpyLimits)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("empy_limits"), true);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasEffortLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}

TEST(TestYAML, testFullLimits)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle("full_limits"), true);

    testPos(limits);
    testVel(limits);
    testAcc(limits);
    testJerk(limits);
    testEffort(limits);
    testSoft(limits);
}

class TestSoftFail : public ::testing::TestWithParam<const char*> {
};

TEST_P(TestSoftFail, testSoftFail)
{
    JointLimiter::Limits limits("test_joint", ros::NodeHandle(GetParam()), true);

    EXPECT_FALSE(limits.hasPositionLimits());
    EXPECT_FALSE(limits.hasVelocityLimits());
    EXPECT_FALSE(limits.hasAccelerationLimits());
    EXPECT_FALSE(limits.hasJerkLimits());
    EXPECT_FALSE(limits.hasEffortLimits());
    EXPECT_FALSE(limits.hasSoftLimits());

    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::PositionLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::VelocityLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::AccelerationLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::JerkLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::EffortLimitsConfigured);
    EXPECT_FALSE(limits.limits_flags & JointLimiter::Limits::SoftLimitsConfigured);
}

INSTANTIATE_TEST_CASE_P(SoftFail,
                        TestSoftFail,
                        ::testing::Values("fail_soft1", "fail_soft2", "fail_soft3","fail_soft4","fail_soft5"));

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "URDFYAMLTestNode");
  return RUN_ALL_TESTS();
}
