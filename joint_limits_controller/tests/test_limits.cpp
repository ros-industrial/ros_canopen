#include <joint_limits_controller/joint_limiter.h>

#include <gtest/gtest.h>

// Declare a test
TEST(TestHelpers, testBounds)
{
    for(int i=-10; i <=10; i+=10){
        EXPECT_EQ( 0+i, JointLimiter::Limits::limitBounds( 0+i, -5+i, +5+i));
        EXPECT_EQ(-5+i, JointLimiter::Limits::limitBounds(-5+i, -5+i, +5+i));
        EXPECT_EQ(-5+i, JointLimiter::Limits::limitBounds(-6+i, -5+i, +5+i));
        EXPECT_EQ(+5+i ,JointLimiter::Limits::limitBounds(+5+i, -5+i, +5+i));
        EXPECT_EQ(+5+i, JointLimiter::Limits::limitBounds(+6+i, -5+i, +5+i));
        EXPECT_TRUE(isnan(JointLimiter::Limits::limitBounds(0, +5+i, -5+i)));
    }
}

TEST(TestHelpers, testSoftLimits)
{
    for(double k=0; k < 2; k+=0.1){
        for(double i=-10; i <=10; i+=1){
            {
                std::pair<double, double> bounds = JointLimiter::Limits::getSoftBounds(0, k,-i,i);
                EXPECT_DOUBLE_EQ( bounds.first, -i*k);
                EXPECT_DOUBLE_EQ( bounds.second, +i*k);
            }
            {
                std::pair<double, double> bounds = JointLimiter::Limits::getSoftBounds(-i, k,-i,i);
                EXPECT_DOUBLE_EQ( bounds.first, 0);
                EXPECT_DOUBLE_EQ( bounds.second, +2*i*k);
            }
            {
                std::pair<double, double> bounds = JointLimiter::Limits::getSoftBounds(+i, k,-i,i);
                EXPECT_DOUBLE_EQ( bounds.first, -2*i*k);
                EXPECT_DOUBLE_EQ( bounds.second, 0);
            }
        }
    }
}

TEST(TestFlags, testNoFlags)
{
    JointLimiter::Limits limits;

    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());
}
TEST(TestFlags, testPosFlag)
{
    // pos
    JointLimiter::Limits limits;
    limits.limits_flags |= JointLimiter::Limits::PositionLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.joint_limits.has_position_limits = true;
    ASSERT_TRUE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.limits_flags &= ~JointLimiter::Limits::PositionLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());
}
TEST(TestFlags, testVelFlag)
{
    // vel
    JointLimiter::Limits limits;
    limits.limits_flags |= JointLimiter::Limits::VelocityLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.joint_limits.has_velocity_limits = true;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_TRUE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.limits_flags &= ~JointLimiter::Limits::VelocityLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());
}
TEST(TestFlags, testAccelFlag)
{
    // accel
    JointLimiter::Limits limits;
    limits.limits_flags |= JointLimiter::Limits::AccelerationLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.joint_limits.has_acceleration_limits = true;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_TRUE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.limits_flags &= ~JointLimiter::Limits::AccelerationLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());
}
TEST(TestFlags, testJerkFlag)
{
    // jerk
    JointLimiter::Limits limits;
    limits.limits_flags |= JointLimiter::Limits::JerkLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.joint_limits.has_jerk_limits = true;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_TRUE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.limits_flags &= ~JointLimiter::Limits::JerkLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());
}
TEST(TestFlags, testEffortFlag)
{
    // effort
    JointLimiter::Limits limits;
    limits.limits_flags |= JointLimiter::Limits::EffortLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.joint_limits.has_effort_limits = true;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_TRUE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.limits_flags &= ~JointLimiter::Limits::EffortLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());
}
TEST(TestFlags, testSoftFlag)
{
    // soft
    JointLimiter::Limits limits;
    limits.limits_flags |= JointLimiter::Limits::SoftLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());

    limits.has_soft_limits = true;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_TRUE(limits.hasSoftLimits());

    limits.limits_flags &= ~JointLimiter::Limits::SoftLimitsConfigured;
    ASSERT_FALSE(limits.hasPositionLimits());
    ASSERT_FALSE(limits.hasVelocityLimits());
    ASSERT_FALSE(limits.hasAccelerationLimits());
    ASSERT_FALSE(limits.hasJerkLimits());
    ASSERT_FALSE(limits.hasEffortLimits());
    ASSERT_FALSE(limits.hasSoftLimits());
}

class TestAccelerationLimits : public ::testing::Test {
protected:
    JointLimiter::Limits limits;
    static const int max_acceleration = 31;
    static const int max_jerk = 13;
    static const int period = 2;
};
TEST_F(TestAccelerationLimits, testNoAccelerationLimits)
{
    double limit=123;
    ASSERT_FALSE(limits.getAccelerationLimit(limit, period));
    ASSERT_EQ(123, limit);
}
TEST_F(TestAccelerationLimits, testOnlyAccelerationLimit)
{
    double limit=123;
    limits.setAccelerationLimits(max_acceleration);

    ASSERT_TRUE(limits.getAccelerationLimit(limit, period));
    ASSERT_DOUBLE_EQ(max_acceleration, limit);
}
TEST_F(TestAccelerationLimits, testOnlyJerkLimit)
{
    double limit=123;
    limits.setJerkLimits(max_jerk);

    ASSERT_TRUE(limits.getAccelerationLimit(limit, period));
    ASSERT_DOUBLE_EQ(max_jerk*period, limit);
}

TEST_F(TestAccelerationLimits, testAccelAndJerkLimit)
{
    double limit=123;
    limits.setAccelerationLimits(max_acceleration);
    limits.setJerkLimits(max_jerk);

    ASSERT_TRUE(limits.getAccelerationLimit(limit, period));
    ASSERT_DOUBLE_EQ(max_jerk*period, limit);

    limits.joint_limits.max_jerk = max_acceleration;

    ASSERT_TRUE(limits.getAccelerationLimit(limit, period));
    ASSERT_DOUBLE_EQ(max_acceleration, limit);
}

class TestVelocityLimits : public TestAccelerationLimits {
protected:
    static const int max_velocity = 83;
};

TEST_F(TestVelocityLimits, testNoVelocityLimits)
{
    double limit=456;
    ASSERT_FALSE(limits.getVelocityLimit(limit, period));
    ASSERT_EQ(456, limit);
}
TEST_F(TestVelocityLimits, testOnlyVelocityLimit)
{
    double limit=456;
    limits.setVelocityLimits(max_velocity);

    ASSERT_TRUE(limits.getVelocityLimit(limit, period));
    ASSERT_DOUBLE_EQ(max_velocity, limit);
}
TEST_F(TestVelocityLimits, testOnlyAccelerationLimit)
{
    double limit=456;

    limits.setAccelerationLimits(max_acceleration);

    ASSERT_TRUE(limits.getVelocityLimit(limit, period));
    ASSERT_DOUBLE_EQ(max_acceleration*period, limit);
}

TEST_F(TestVelocityLimits, testVelocityAndAccelerationLimit)
{
    double limit=456;

    limits.setVelocityLimits(max_velocity);
    limits.setAccelerationLimits(max_acceleration);

    ASSERT_TRUE(limits.getVelocityLimit(limit, period));
    ASSERT_DOUBLE_EQ(max_acceleration*period, limit);

    limits.joint_limits.max_acceleration = max_velocity;

    ASSERT_TRUE(limits.getVelocityLimit(limit, period));
    ASSERT_DOUBLE_EQ(max_velocity, limit);
}

TEST(TestVelocitySoftLimits, testNoVelocitySoftLimits)
{
    JointLimiter::Limits limits;
    std::pair<double, double> bounds = limits.getVelocitySoftBounds(0);
    EXPECT_TRUE( isnan(bounds.first));
    EXPECT_TRUE( isnan(bounds.second));
}

TEST(TestVelocitySoftLimits, testVelocitySoftLimits)
{
    JointLimiter::Limits limits;

    limits.setSoftLimits(2.0, -10.0, +10.0, 0.0);

    {
        std::pair<double, double> bounds = limits.getVelocitySoftBounds(0);
        EXPECT_DOUBLE_EQ( -10*2,bounds.first);
        EXPECT_DOUBLE_EQ( +10*2,bounds.second);
    }

    limits.setVelocityLimits(5.0);

    {
        std::pair<double, double> bounds = limits.getVelocitySoftBounds(0);
        EXPECT_DOUBLE_EQ( -5,bounds.first);
        EXPECT_DOUBLE_EQ( +5,bounds.second);
    }
}

TEST(TestHardLimits, testPositionHardLimits)
{
    JointLimiter::Limits limits;

    limits.setPositionLimits(-5.0, 10.0);

    for(double i=2*limits.joint_limits.min_position; i <= 2*limits.joint_limits.max_position; i+=1.0){
        if( i >= limits.joint_limits.max_position){
            EXPECT_EQ(limits.joint_limits.max_position, limits.limitPosition(i));
        }else if(i <= limits.joint_limits.min_position ){
            EXPECT_EQ(limits.joint_limits.min_position, limits.limitPosition(i));
        }else{
            EXPECT_DOUBLE_EQ(i, limits.limitPosition(i));
        }
    }

    limits.limits_flags &= ~JointLimiter::Limits::PositionLimitsConfigured;

    for(double i=2*limits.joint_limits.min_position; i <= 2*limits.joint_limits.max_position; i+=1.0){
        EXPECT_DOUBLE_EQ(i, limits.limitPosition(i));
    }
}

TEST(TestHardLimits, testVelocityHardLimits)
{
    JointLimiter::Limits limits;

    limits.setVelocityLimits(5.0);

    for(double i=-2*limits.joint_limits.max_velocity; i <= 2*limits.joint_limits.max_velocity; i+=1.0){
        if(fabs(i) <= limits.joint_limits.max_velocity){
            EXPECT_DOUBLE_EQ(i, limits.limitVelocity(i));
        }else if( i> 0){
            EXPECT_EQ(limits.joint_limits.max_velocity, limits.limitVelocity(i));
        }else{
            EXPECT_EQ(-limits.joint_limits.max_velocity, limits.limitVelocity(i));
        }
    }

    limits.limits_flags &= ~JointLimiter::Limits::VelocityLimitsConfigured;

    for(double i=-2*limits.joint_limits.max_velocity; i <= 2*limits.joint_limits.max_velocity; i+=1.0){
        EXPECT_DOUBLE_EQ(i, limits.limitVelocity(i));
    }
}

TEST(TestHardLimits, testEffortHardLimits)
{
    JointLimiter::Limits limits;

    limits.setEffortLimits(5.0);

    for(double i=-2*limits.joint_limits.max_effort; i <= 2*limits.joint_limits.max_effort; i+=1.0){
        if(fabs(i) <= limits.joint_limits.max_effort){
            EXPECT_DOUBLE_EQ(i, limits.limitEffort(i));
        }else if( i> 0){
            EXPECT_EQ(limits.joint_limits.max_effort, limits.limitEffort(i));
        }else{
            EXPECT_EQ(-limits.joint_limits.max_effort, limits.limitEffort(i));
        }
    }
    limits.limits_flags &= ~JointLimiter::Limits::EffortLimitsConfigured;

    for(double i=-2*limits.joint_limits.max_effort; i <= 2*limits.joint_limits.max_effort; i+=1.0){
        EXPECT_DOUBLE_EQ(i, limits.limitEffort(i));
    }
}

TEST(TestHardLimits, testStopOnPositionLimit)
{
    JointLimiter::Limits limits;

    limits.setPositionLimits(-5.0, 10.0);

    for(double i=2*limits.joint_limits.min_position; i <= 2*limits.joint_limits.max_position; i+=1.0){
        if( i > limits.joint_limits.max_position){
            EXPECT_EQ(-1, limits.stopOnPositionLimit(-1, i));
            EXPECT_EQ( 0, limits.stopOnPositionLimit(1, i));
        }else if(i < limits.joint_limits.min_position ){
            EXPECT_EQ( 0, limits.stopOnPositionLimit(-1, i));
            EXPECT_EQ(+1, limits.stopOnPositionLimit(1, i));
        }else{
            EXPECT_DOUBLE_EQ(-1, limits.stopOnPositionLimit(-1,i));
            EXPECT_DOUBLE_EQ(+1, limits.stopOnPositionLimit(+1,i));
        }
        EXPECT_EQ(0, limits.stopOnPositionLimit(0, i));
    }

    limits.limits_flags &= ~JointLimiter::Limits::PositionLimitsConfigured;

    for(double i=2*limits.joint_limits.min_position; i <= 2*limits.joint_limits.max_position; i+=1.0){
        EXPECT_DOUBLE_EQ( 0, limits.stopOnPositionLimit( 0,i));
        EXPECT_DOUBLE_EQ(-1, limits.stopOnPositionLimit(-1,i));
        EXPECT_DOUBLE_EQ(+1, limits.stopOnPositionLimit(+1,i));
    }

}

TEST(TestMerge, testEmptyMerge)
{
    JointLimiter::Limits limits1, limits2;
    JointLimiter::Limits limits3(limits1, limits2);

    ASSERT_EQ(limits1, limits2);
    ASSERT_EQ(limits1, limits3);
    ASSERT_EQ(limits2, limits3);
}

TEST(TestMerge, testPositionMerge)
{
    JointLimiter::Limits limits1, limits2, limits3;

    limits2.setPositionLimits(-10,10);

    limits1.merge(limits2);
    ASSERT_EQ(limits1, limits2);

    limits3.setPositionLimits(-20,10);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setPositionLimits(-10,20);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setPositionLimits(-20,20);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setPositionLimits(-10,5);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);

    limits3.setPositionLimits(-5,5);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);

}

TEST(TestMerge, testVelocityMerge)
{
    JointLimiter::Limits limits1, limits2, limits3;

    limits2.setVelocityLimits(10);

    limits1.merge(limits2);
    ASSERT_EQ(limits1, limits2);

    limits3.setVelocityLimits(20);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setVelocityLimits(5);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);
}

TEST(TestMerge, testAccelerationMerge)
{
    JointLimiter::Limits limits1, limits2, limits3;

    limits2.setAccelerationLimits(10);

    limits1.merge(limits2);
    ASSERT_EQ(limits1, limits2);

    limits3.setAccelerationLimits(20);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setAccelerationLimits(5);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);
}

TEST(TestMerge, testJerkMerge)
{
    JointLimiter::Limits limits1, limits2, limits3;

    limits2.setJerkLimits(10);

    limits1.merge(limits2);
    ASSERT_EQ(limits1, limits2);

    limits3.setJerkLimits(20);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setJerkLimits(5);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);

}

TEST(TestMerge, testEffortMerge)
{
    JointLimiter::Limits limits1, limits2, limits3;

    limits2.setEffortLimits(10);

    limits1.merge(limits2);
    ASSERT_EQ(limits1, limits2);

    limits3.setEffortLimits(20);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setEffortLimits(5);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);

}

TEST(TestMerge, testSoftLimitsMerge)
{
    JointLimiter::Limits limits1, limits2, limits3;

    limits2.setSoftLimits(2,-10, 10, 3);

    limits1.merge(limits2);
    ASSERT_EQ(limits1, limits2);

    limits3.setSoftLimits(3,-10, 10, 3);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setSoftLimits(2,-20, 10, 3);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setSoftLimits(2,-10, 20, 3);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setSoftLimits(2,-10, 10, 4);
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.setSoftLimits(1,-10, 10, 3);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);

    limits3.setSoftLimits(1,-5, 10, 3);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);

    limits3.setSoftLimits(1,-5, 5, 3);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);

    limits3.setSoftLimits(1,-5, 5, 2);
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_EQ(limits1, limits3);
}

TEST(TestMerge, testNoMerge)
{
    JointLimiter::Limits limits1, limits2, limits3;

    limits1.setPositionLimits(0,0);
    limits1.setVelocityLimits(0);
    limits1.setAccelerationLimits(0);
    limits1.setJerkLimits(0);
    limits1.setEffortLimits(0);
    limits1.setSoftLimits(0,0,0,0);

    limits2 = limits1;

    ASSERT_EQ(limits1, limits2);

    limits3.limits_flags = JointLimiter::Limits::PositionLimitsConfigured;
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.limits_flags = JointLimiter::Limits::VelocityLimitsConfigured;
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.limits_flags = JointLimiter::Limits::AccelerationLimitsConfigured;
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.limits_flags = JointLimiter::Limits::JerkLimitsConfigured;
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.limits_flags = JointLimiter::Limits::EffortLimitsConfigured;
    limits1.merge(limits3);
    ASSERT_EQ(limits1, limits2);
    ASSERT_NE(limits1, limits3);

    limits3.limits_flags = JointLimiter::Limits::SoftLimitsConfigured;
    limits1.merge(limits3);
    ASSERT_NE(limits1, limits2);
    ASSERT_FALSE(limits1.hasSoftLimits());
}

TEST(TestApply, testApply)
{
    JointLimiter::Limits limits1, limits2, limits3;

    limits1.setPositionLimits(0,0);
    limits1.setVelocityLimits(0);
    limits1.setAccelerationLimits(0);
    limits1.setJerkLimits(0);
    limits1.setEffortLimits(0);
    limits1.setSoftLimits(0,0,0,0);

    limits2 = limits1;

    ASSERT_EQ(limits1, limits2);

    limits1.apply(limits3);

    ASSERT_EQ(limits1, limits2);

    limits3.limits_flags = JointLimiter::Limits::PositionLimitsConfigured;
    ASSERT_TRUE(limits1.hasPositionLimits());
    limits1.apply(limits3);
    ASSERT_FALSE(limits1.hasPositionLimits());

    limits3.limits_flags = JointLimiter::Limits::VelocityLimitsConfigured;
    ASSERT_TRUE(limits1.hasVelocityLimits());
    limits1.apply(limits3);
    ASSERT_FALSE(limits1.hasVelocityLimits());

    limits3.limits_flags = JointLimiter::Limits::AccelerationLimitsConfigured;
    ASSERT_TRUE(limits1.hasAccelerationLimits());
    limits1.apply(limits3);
    ASSERT_FALSE(limits1.hasAccelerationLimits());

    limits3.limits_flags = JointLimiter::Limits::JerkLimitsConfigured;
    ASSERT_TRUE(limits1.hasJerkLimits());
    limits1.apply(limits3);
    ASSERT_FALSE(limits1.hasJerkLimits());

    limits3.limits_flags = JointLimiter::Limits::EffortLimitsConfigured;
    ASSERT_TRUE(limits1.hasEffortLimits());
    limits1.apply(limits3);
    ASSERT_FALSE(limits1.hasEffortLimits());

    limits3.limits_flags = JointLimiter::Limits::SoftLimitsConfigured;
    ASSERT_TRUE(limits1.hasSoftLimits());
    limits1.apply(limits3);
    ASSERT_FALSE(limits1.hasSoftLimits());

    limits3.setPositionLimits(1,2);
    limits3.setVelocityLimits(3);
    limits3.setAccelerationLimits(4);
    limits3.setJerkLimits(5);
    limits3.setEffortLimits(6);
    limits3.setSoftLimits(7,8,9,10);

    limits1.apply(limits3);

    ASSERT_EQ(limits1, limits3);
    ASSERT_TRUE(limits1.hasPositionLimits());
    ASSERT_TRUE(limits1.hasVelocityLimits());
    ASSERT_TRUE(limits1.hasAccelerationLimits());
    ASSERT_TRUE(limits1.hasJerkLimits());
    ASSERT_TRUE(limits1.hasEffortLimits());
    ASSERT_TRUE(limits1.hasSoftLimits());
}

// TODO: add unit tests for valid()

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
