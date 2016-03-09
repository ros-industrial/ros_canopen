#include <joint_limits_controller/joint_limiter.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

bool use_real_time = false;
double loop_rate = 50;

const double EPS = 1e-12;
template<typename T> class MinMaxStore {
    DataStore<T> min_;
    DataStore<T> max_;
    DataStore<T> val_;
public:
    void set(const T &data){
        val_.set(data);
        min_.setMin(data);
        max_.setMax(data);
    }
    bool getMinMax(double &min, double &max){
        return min_.get(min) && max_.get(max);
    }
    bool getVal(double &val){
        return val_.get(val);
    }
};

class Analyzer {
protected:
    MinMaxStore<double> minmax_;
    MinMaxStore<double> first_;
    MinMaxStore<double> second_;
    MinMaxStore<double> third_;

    DataStore<double> data_[3];

public:
    bool getMinMax(double &min, double &max){
        return minmax_.getMinMax(min, max);
    }
    bool getFirstMinMax(double &min, double &max){
        return first_.getMinMax(min, max);
    }
    bool getSecondMinMax(double &min, double &max){
        return second_.getMinMax(min, max);
    }
    bool getThirdMinMax(double &min, double &max){
        return third_.getMinMax(min, max);
    }
    bool getVals(double &val, double &first, double &second, double &third){
        return minmax_.getVal(val) && first_.getVal(first) && second_.getVal(second) && third_.getVal(third);
    }
    void update(double val, double dt){

        double old[3];

        minmax_.set(val);

        if(data_[0].get(old[0]) && data_[1].get(old[1])){
            first_.set((val - old[1])/(2*dt));
            second_.set((val - 2*old[0] + old[1])/(dt*dt));

            if(data_[2].get(old[2])){
                third_.set((val - 3*old[0] + 3 * old[1] - old[2])/(dt*dt*dt));
            }

        }

        data_[2] = data_[1];
        data_[1] = data_[0];
        data_[0].set(val);
    }
};

template<typename Limiter> class Runner : public ::testing::Test {
protected:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    std_msgs::Float64MultiArray data_;

    Analyzer ana_;
    JointLimiter::Limits limits_;
    Limiter limiter_;
    MinMaxStore<double> error_;

    double time_, dt_;

    virtual void SetUp() {
        const ::testing::TestInfo* const test_info =::testing::UnitTest::GetInstance()->current_test_info();
        std::cerr << "Running " << test_info->test_case_name() << "." << test_info->name() << std::endl;
    }

    virtual double control() = 0;
    virtual void check() {}

    void run(double rate, double duration, bool real_time){
        if(real_time) ros::Rate(1.0).sleep();
        ros::Rate r(rate);
        dt_ = 1/rate;
        time_ = 0;
        ros::Duration dur(duration);
        ros::Time start = ros::Time::now();
        while(ros::ok() && (ros::Time::now() - start) <= dur && time_ <= duration){
            data_.data[0] = control();

            data_.data[1] = data_.data[0];
            limiter_.enforceLimits(dt_,limits_, data_.data[3], data_.data[4], data_.data[5], data_.data[1]);

            pub_.publish(data_);

            ana_.update(data_.data[1], dt_);
            data_.data[2] = data_.data[1]-data_.data[0];
            error_.set(data_.data[2]);

            check();

            ros::spinOnce();
            if(real_time) r.sleep();
            time_+=dt_;
        }
    }
    virtual void stats(){
        double min, max;
        ASSERT_TRUE(ana_.getMinMax(min,max));
        std::cerr << min << " command " << max << std::endl;

        ASSERT_TRUE(ana_.getFirstMinMax(min,max));
        std::cerr << min << " first " << max << std::endl;

        ASSERT_TRUE(ana_.getSecondMinMax(min,max));
        std::cerr << min << " second " << max << std::endl;

        ASSERT_TRUE(ana_.getThirdMinMax(min,max));
        std::cerr << min << " third " << max << std::endl;

        ASSERT_TRUE(error_.getMinMax(min, max));
        std::cerr << min << " error " << max << std::endl;
    }
public:
    Runner(){
        pub_ = nh_.advertise<std_msgs::Float64MultiArray>("runner",1);
        data_.data.resize(3+3);
    }
    virtual ~Runner() {}
};

template<typename Limiter> class SineRunner : public Runner<Limiter> {
protected:
    double amplitude_, frequency_;
    virtual void init(double amplitude, double period) {
        amplitude_ = amplitude; frequency_ = 2*M_PI/period;
    }
    virtual double control() { return amplitude_ * sin(this->time_ * frequency_); }
};

class PosSineRunner: public SineRunner<PositionJointLimiter>{
protected:
    virtual void init(double amplitude, double period) {
        SineRunner::init(amplitude, period);
        this->data_.data[4] = limits_.limitVelocity(amplitude_ * frequency_);
    }
    virtual void check(){
        double acc, jerk;
        ana_.getVals(data_.data[3], data_.data[4], acc, jerk);
        ASSERT_NEAR (this->data_.data[3], this->limits_.limitPosition(this->data_.data[3]), EPS);
        ASSERT_NEAR (this->data_.data[4], this->limits_.limitVelocity(this->data_.data[4]), EPS);
        ASSERT_NEAR (acc, this->limits_.limitAcceleration(acc), 1e-11);
        ASSERT_NEAR (jerk, this->limits_.limitJerk(jerk), 1e-5);
    }
};

TEST_F(PosSineRunner, testPosSineNoLimits)
{
    init(1,2*M_PI);
    run(loop_rate, 10, use_real_time);
    stats();

    double min_err, max_err;
    ASSERT_TRUE(error_.getMinMax(min_err, max_err));

    ASSERT_NEAR(0.0, min_err, EPS);
    ASSERT_NEAR(0.0, max_err, EPS);
}

TEST_F(PosSineRunner, testPosSineHighLimits)
{

    limits_.setPositionLimits(-1, 1);
    limits_.setVelocityLimits(1);
    init(1,2*M_PI);
    run(loop_rate, 10, use_real_time);
    stats();

    double min_err, max_err;
    ASSERT_TRUE(error_.getMinMax(min_err, max_err));

    ASSERT_NEAR(0.0, min_err, EPS);
    ASSERT_NEAR(0.0, max_err, EPS);
}

TEST_F(PosSineRunner, testPosSineHighLimitsFast)
{

    limits_.setPositionLimits(-10, 10);
    limits_.setVelocityLimits(20);
    init(10,M_PI);
    run(loop_rate, 10, use_real_time);
    stats();

    double min_err, max_err;
    ASSERT_TRUE(error_.getMinMax(min_err, max_err));

    ASSERT_NEAR(0.0, min_err, EPS);
    ASSERT_NEAR(0.0, max_err, EPS);
}

TEST_F(PosSineRunner, testPosSinePosBound)
{

    limits_.setPositionLimits(-3, 7);
    init(10, 4);
    run(loop_rate, 10, use_real_time);
    stats();

    double min_err, max_err;
    ASSERT_TRUE(error_.getMinMax(min_err, max_err));

    ASSERT_NEAR(-3.0, min_err, 1e-4);
    ASSERT_NEAR(7.0, max_err, 1e-4);
}

TEST_F(PosSineRunner, testPosSineVelocityLimits)
{

    limits_.setVelocityLimits(5);
    init(10, 4);
    run(loop_rate, 10, use_real_time);
    stats();
}

class VelSineRunner: public SineRunner<VelocityJointLimiter>{
protected:
    MinMaxStore<double> pos_;
    void init(double amplitude, double period) {
        SineRunner::init(amplitude, period);
        data_.data[3] = limits_.limitPosition(-amplitude_ /frequency_);
    }
    virtual void check(){
        double acc, jerk, jounce;
        ana_.getVals(data_.data[4], acc, jerk, jounce);
        this->data_.data[3] += this->data_.data[4]*dt_;
        pos_ .set(this->data_.data[3]);
        ASSERT_NEAR (this->data_.data[3], this->limits_.limitPosition(this->data_.data[3]), 1e-4);
        ASSERT_NEAR (this->data_.data[4], this->limits_.limitVelocity(this->data_.data[4]), EPS);
        ASSERT_NEAR (acc, this->limits_.limitAcceleration(acc), 1e-11);
        ASSERT_NEAR (jerk, this->limits_.limitJerk(jerk), 1e-5);
    }
    virtual void stats(){
        double min, max;
        ASSERT_TRUE(pos_.getMinMax(min, max));
        std::cerr << min << " pos " << max << std::endl;
        Runner::stats();
    }
};

TEST_F(VelSineRunner, testVelSineNoLimits)
{
    init(1,2*M_PI);
    run(loop_rate, 10, use_real_time);
    stats();

    double min_err, max_err;
    ASSERT_TRUE(error_.getMinMax(min_err, max_err));

    ASSERT_NEAR(0.0, min_err, EPS);
    ASSERT_NEAR(0.0, max_err, EPS);
}

TEST_F(VelSineRunner, testVelSineHighLimits)
{

    limits_.setVelocityLimits(1);
    limits_.setAccelerationLimits(1);
    init(1,2*M_PI);
    run(loop_rate, 10, use_real_time);
    stats();

    double min_err, max_err;
    ASSERT_TRUE(error_.getMinMax(min_err, max_err));

    ASSERT_NEAR(0.0, min_err, EPS);
    ASSERT_NEAR(0.0, max_err, EPS);
}

TEST_F(VelSineRunner, testVelSineHighLimitsFast)
{

    //limits_.setPositionLimits(-10, 10);
    limits_.setVelocityLimits(10);
    limits_.setAccelerationLimits(20);
    init(1,M_PI);
    run(loop_rate, 10, use_real_time);
    stats();

    double min_err, max_err;
    ASSERT_TRUE(error_.getMinMax(min_err, max_err));

    ASSERT_NEAR(0.0, min_err, EPS);
    ASSERT_NEAR(0.0, max_err, EPS);
}

TEST_F(VelSineRunner, testVelSinePosBound)
{

    limits_.setPositionLimits(-0.30, 0.70);
    init(1, 2*M_PI);
    data_.data[3] = -0.3;
    run(loop_rate, 10, use_real_time);
    stats();
}
TEST_F(VelSineRunner, testVelSinePosSoftBound)
{

    limits_.setSoftLimits(10,-0.30, 0.70,0);
    init(1, 2*M_PI);
    data_.data[3] = -0.3;
    run(loop_rate, 10, use_real_time);
    stats();

}

template<typename Limiter> class RectRunner : public Runner <Limiter>{
    double start_value_;
    double end_value_;
    double time1_, time2_;

protected:
    void init(double start, double time1, double end, double time2){
        start_value_ = start;
        end_value_ = end;
        time1_ = time1;
        time2_ = time2;
    }
    virtual double control() {
        return (this->time_ < time1_ || this->time_ >= time2_) ? start_value_ : end_value_;
    }
};

class PosRectRunner: public RectRunner<PositionJointLimiter>{
protected:
    virtual void check(){
        double acc, jerk;
        ana_.getVals(data_.data[3], data_.data[4], acc, jerk);
        ASSERT_NEAR (this->data_.data[3], this->limits_.limitPosition(this->data_.data[3]), EPS);
        ASSERT_NEAR (this->data_.data[4], this->limits_.limitVelocity(this->data_.data[4]), EPS);
        ASSERT_NEAR (acc, this->limits_.limitAcceleration(acc), 1e-11);
        ASSERT_NEAR (jerk, this->limits_.limitJerk(jerk), 1e-5);
    }
};

TEST_F(PosRectRunner, TestPosJumpNoLimit)
{
    init(0, 5, 10, 10);
    run(loop_rate, 15, use_real_time);
    stats();
}

TEST_F(PosRectRunner, TestPosJumpVelLimit)
{
    limits_.setVelocityLimits(5);
    init(0, 5, 10, 10);
    run(loop_rate, 15, use_real_time);
    stats();
}
TEST_F(PosRectRunner, TestPosJumpVelLimitNeg)
{
    limits_.setVelocityLimits(5);
    init(0, 5, -10, 10);
    run(loop_rate, 15, use_real_time);
    stats();
}

class VelRectRunner: public RectRunner<VelocityJointLimiter>{
protected:
    MinMaxStore<double> pos_;
    virtual void check(){
        double acc, jerk, jounce;
        ana_.getVals(data_.data[4], acc, jerk, jounce);
        this->data_.data[3] += this->data_.data[4]*dt_;
        pos_ .set(this->data_.data[3]);
        ASSERT_NEAR (this->data_.data[3], this->limits_.limitPosition(this->data_.data[3]), 1e-4);
        ASSERT_NEAR (this->data_.data[4], this->limits_.limitVelocity(this->data_.data[4]), EPS);
        ASSERT_NEAR (acc, this->limits_.limitAcceleration(acc), 1e-11);
        ASSERT_NEAR (jerk, this->limits_.limitJerk(jerk), 1e-5);
    }
    virtual void stats(){
        double min, max;
        ASSERT_TRUE(pos_.getMinMax(min, max));
        std::cerr << min << " pos " << max << std::endl;
        Runner::stats();
    }
};

TEST_F(VelRectRunner, TestVelJumpNoLimit)
{
    init(0, 5, 10, 10);
    run(loop_rate, 15, use_real_time);
    stats();
}

TEST_F(VelRectRunner, TestVelJumpAccLimit)
{
    limits_.setAccelerationLimits(1);
    init(0, 5, 10, 10);
    run(loop_rate, 15, use_real_time);
    stats();
}
TEST_F(VelRectRunner, TestVelJumpAccLimitNeg)
{
    limits_.setAccelerationLimits(1);
    init(0, 5, -10, 10);
    run(loop_rate, 15, use_real_time);
    stats();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "JointLimiterTestNode");
  ros::NodeHandle nh_priv("~");
  nh_priv.getParam("use_real_time", use_real_time);
  nh_priv.getParam("loop_rate", loop_rate);
  return RUN_ALL_TESTS();
}
