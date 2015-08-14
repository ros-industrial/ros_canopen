#include <canopen_402/motor.h>
#include <gtest/gtest.h>


template<typename T> class ModeTargetHelperTest : public canopen::ModeTargetHelper<T>, public ::testing::Test{
public:
    ModeTargetHelperTest() : canopen::ModeTargetHelper<T>(0) {}
    virtual bool read(const uint16_t &sw) { return false; }
    virtual bool write(canopen::Mode::OpModeAccesser& cw) { return false; }
};

typedef ::testing::Types<uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t, uint64_t, int64_t> MyTypes;

TYPED_TEST_CASE(ModeTargetHelperTest, MyTypes);

TYPED_TEST(ModeTargetHelperTest, CheckNaN){
    ASSERT_FALSE(this->setTarget(std::numeric_limits<double>::quiet_NaN()));
}

TYPED_TEST(ModeTargetHelperTest, CheckZero){
    ASSERT_TRUE(this->setTarget(0.0));
}

TYPED_TEST(ModeTargetHelperTest, CheckOne){
    ASSERT_TRUE(this->setTarget(1.0));
}

TYPED_TEST(ModeTargetHelperTest, CheckMax){
    double max = static_cast<double>(std::numeric_limits<TypeParam>::max());

    ASSERT_TRUE(this->setTarget(max));
    ASSERT_EQ(max, this->getTarget());

    ASSERT_TRUE(this->setTarget(max-1));
    ASSERT_EQ(max-1,this->getTarget());

    ASSERT_TRUE(this->setTarget(max+1));
    ASSERT_EQ(max, this->getTarget());
}

TYPED_TEST(ModeTargetHelperTest, CheckMin){
    double min = static_cast<double>(std::numeric_limits<TypeParam>::min());

    ASSERT_TRUE(this->setTarget(min));
    ASSERT_EQ(min, this->getTarget());

    ASSERT_TRUE(this->setTarget(min-1));
    ASSERT_EQ(min, this->getTarget());

    ASSERT_TRUE(this->setTarget(min+1));
    ASSERT_EQ(min+1,this->getTarget());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
