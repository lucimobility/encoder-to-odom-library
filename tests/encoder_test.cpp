#include "encoder_to_odom/odometry.h"
#include <gtest/gtest.h>

constexpr float WHEEL_CIRCUMFERENCE = 1.0373;
constexpr float WHEEL_BASE = 0.5065;  // 0.4926;  // 0.56 // 0.5326
constexpr float GEAR_RATIO = 2.38462; // 500 prototype -> 2.1138 (761 turns per wheel revolution)
constexpr float ROLLOVER = 100.0;
constexpr bool RIGHT_INCREASE = true;
constexpr bool LEFT_INCREASE = false;

class Tester : public OdometryProcessor
{
  public:
    Tester()
        : OdometryProcessor(WHEEL_CIRCUMFERENCE, WHEEL_BASE, GEAR_RATIO, ROLLOVER, RIGHT_INCREASE,
                            LEFT_INCREASE)
    {
    }

    void callDegreeCalculation()
    {
        this->calculateDegreesTraveledInFrame(Motor::LEFT);
        this->calculateDegreesTraveledInFrame(Motor::RIGHT);
    }
    void callMeterCalculation()
    {
        this->calculateMetersMotorTraveledInFrame(Motor::LEFT);
        this->calculateMetersMotorTraveledInFrame(Motor::RIGHT);
    }

    void callFrameDistanceCalculation() { this->calculateFrameDistance(); }

    void callThetaCalculation() { this->calculateTheta(); }

    void settleReadings(float leftValue, float rightValue)
    {
        for (int i = 0; i < SETTLE_READINGS; i++)
        {
            this->updateCurrentValue(Motor::LEFT, leftValue);
            this->updateCurrentValue(Motor::RIGHT, rightValue);
            this->timestamp = 1000 * (i + 1);
            this->updateTimestamp(this->timestamp); // Each reading happens one second apart
            this->processData();
        }
    }

    void driveFullEncoderRotation()
    {
        float leftEncoderReading = 120;
        float rightEncoderReading = 300;
        this->settleReadings(leftEncoderReading, rightEncoderReading);

        this->updateCurrentValue(Motor::LEFT, 350);  // -130
        this->updateCurrentValue(Motor::RIGHT, 100); // 160
        this->timestamp += 1000;
        this->updateTimestamp(this->timestamp); // Each reading happens one second apart

        this->processData();

        this->updateCurrentValue(Motor::LEFT, 260);  // -90
        this->updateCurrentValue(Motor::RIGHT, 190); // 90
        this->timestamp += 1000;
        this->updateTimestamp(this->timestamp); // Each reading happens one second apart
        this->processData();

        this->updateCurrentValue(Motor::LEFT, 160);  // -100
        this->updateCurrentValue(Motor::RIGHT, 290); // 100
        this->timestamp += 1000;
        this->updateTimestamp(this->timestamp); // Each reading happens one second apart
        this->processData();

        this->updateCurrentValue(Motor::LEFT, 120);  // 40
        this->updateCurrentValue(Motor::RIGHT, 300); // 10
        this->timestamp += 1000;
        this->updateTimestamp(this->timestamp); // Each reading happens one second apart
        this->processData();
    }

    void driveStraightOneEncoderRotation()
    {
        float leftEncoderReading = 120;
        float rightEncoderReading = 300;
        this->settleReadings(leftEncoderReading, rightEncoderReading);

        this->updateCurrentValue(Motor::LEFT, 350); // -130
        this->updateCurrentValue(Motor::RIGHT, 70); // 130
        this->processData();

        this->updateCurrentValue(Motor::LEFT, 260);  // -90
        this->updateCurrentValue(Motor::RIGHT, 160); // 90
        this->processData();

        this->updateCurrentValue(Motor::LEFT, 160);  // -100
        this->updateCurrentValue(Motor::RIGHT, 260); // 100
        this->processData();

        this->updateCurrentValue(Motor::LEFT, 120);  // 40
        this->updateCurrentValue(Motor::RIGHT, 300); // 40
        this->processData();
    }

    uint16_t startTime = 0;
};

TEST(FrameTests, UpdateReadings)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 50;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

    // processor.updateCurrentValue(Motor::LEFT, leftEncoderReading);
    // processor.updateCurrentValue(Motor::RIGHT, rightEncoderReading);

    auto currentLeftReading = processor.getCurrentReading(Motor::LEFT);
    auto currentRightReading = processor.getCurrentReading(Motor::RIGHT);

    ASSERT_EQ(leftEncoderReading, currentLeftReading);
    ASSERT_EQ(rightEncoderReading, currentRightReading);
}

TEST(FrameTests, Settled)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 300;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

    processor.callDegreeCalculation();

    auto totalDegreesLeft = processor.getTotalDegreesTraveled(Motor::LEFT);

    ASSERT_EQ(0.0, totalDegreesLeft);
}

TEST(FrameTests, DeltaDegrees)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 50;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

    // processor.updateCurrentValue(Motor::LEFT, leftEncoderReading);
    // processor.updateCurrentValue(Motor::RIGHT, rightEncoderReading);

    leftEncoderReading = 90;
    rightEncoderReading = 70;

    processor.updateCurrentValue(Motor::LEFT, leftEncoderReading);
    processor.updateCurrentValue(Motor::RIGHT, rightEncoderReading);

    processor.callDegreeCalculation();

    auto totalDegreesLeft = processor.getTotalDegreesTraveled(Motor::LEFT);
    auto totalDegreesRight = processor.getTotalDegreesTraveled(Motor::RIGHT);

    ASSERT_EQ(-30.0, totalDegreesLeft);
    ASSERT_EQ(20.0, totalDegreesRight);
}

TEST(FrameTests, RollOver)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 300;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

    // processor.updateCurrentValue(Motor::LEFT, leftEncoderReading);
    // processor.updateCurrentValue(Motor::RIGHT, rightEncoderReading);

    leftEncoderReading = 350; // Delta of -130
    rightEncoderReading = 50; // Delta of 110 degrees

    processor.updateCurrentValue(Motor::LEFT, leftEncoderReading);
    processor.updateCurrentValue(Motor::RIGHT, rightEncoderReading);

    processor.callDegreeCalculation();

    auto totalDegreesLeft = processor.getTotalDegreesTraveled(Motor::LEFT);
    auto totalDegreesRight = processor.getTotalDegreesTraveled(Motor::RIGHT);

    ASSERT_EQ(-130.0, totalDegreesLeft);
    ASSERT_EQ(110.0, totalDegreesRight);
}

// One full rotation on encoders
TEST(FrameTests, DeltaMetersOneEncoderRotation)
{
    auto processor = Tester();

    processor.driveFullEncoderRotation();

    auto totalDegreesLeft = processor.getTotalDegreesTraveled(Motor::LEFT);
    auto totalDegreesRight = processor.getTotalDegreesTraveled(Motor::RIGHT);

    ASSERT_EQ(-360.0, totalDegreesLeft);
    ASSERT_EQ(360.0, totalDegreesRight);

    auto metersTraveledLeft = processor.getTotalMetersTraveled(Motor::LEFT);
    auto metersTraveledRight = processor.getTotalMetersTraveled(Motor::RIGHT);

    ASSERT_NEAR(WHEEL_CIRCUMFERENCE / GEAR_RATIO, metersTraveledLeft, 0.01);
    ASSERT_NEAR(WHEEL_CIRCUMFERENCE / GEAR_RATIO, metersTraveledRight, 0.01);
}

TEST(TotalTests, AngleTest)
{
    auto processor = Tester();

    processor.driveFullEncoderRotation();
    // processor.calculateDistanceMovedX();
    // processor.callFrameDistanceCalculation();
    // processor.callThetaCalculation();

    auto position = processor.getPosition();

    ASSERT_NEAR(0.0, position.theta, 0.001);
}

TEST(TotalTests, PositionTest)
{
    auto processor = Tester();

    processor.driveStraightOneEncoderRotation();

    auto totalDegreesLeft = processor.getTotalDegreesTraveled(Motor::LEFT);
    auto totalDegreesRight = processor.getTotalDegreesTraveled(Motor::RIGHT);

    ASSERT_EQ(-360.0, totalDegreesLeft);
    ASSERT_EQ(360.0, totalDegreesRight);
    // processor.calculateDistanceMovedX();
    // processor.callFrameDistanceCalculation();
    // processor.callThetaCalculation();

    auto position = processor.getPosition();

    float correctXPosition = WHEEL_CIRCUMFERENCE / GEAR_RATIO;

    ASSERT_NEAR(0, position.y, 0.01);
    ASSERT_NEAR(correctXPosition, position.x, 0.01);
}

TEST(FrameTests, Velocity)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 50;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

    // processor.updateCurrentValue(Motor::LEFT, leftEncoderReading);
    // processor.updateCurrentValue(Motor::RIGHT, rightEncoderReading);

    leftEncoderReading = 90;
    rightEncoderReading = 70;

    processor.updateCurrentValue(Motor::LEFT, leftEncoderReading);
    processor.updateCurrentValue(Motor::RIGHT, rightEncoderReading);

    processor.updateTimestamp(1000); // Each reading happens one second apart
    processor.processData();

    auto distanceInFrame = processor.getDistance().frameDistance;
    auto deltaAngle = processor.getPosition().theta;
    auto calculatedVelocity = processor.getVelocity();

    ASSERT_EQ(distanceInFrame, calculatedVelocity.linearY); // Meters / sec
    ASSERT_EQ(deltaAngle, calculatedVelocity.angularZ);     // Rad / sec
}

// TODO: clp make this auto run on make -jn call
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}