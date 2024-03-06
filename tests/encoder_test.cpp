#include "encoder_to_odom/odometry.h"
#include <gtest/gtest.h>

// Default test values
constexpr float WHEEL_CIRCUMFERENCE = 1.0373;
constexpr float WHEEL_BASE = 0.5065;
constexpr float GEAR_RATIO = 2.38462;
constexpr float ROLLOVER = 100.0;
constexpr bool RIGHT_INCREASE = true;
constexpr bool LEFT_INCREASE = false;

/**
 * @brief Wrapper class to allow calling protected functions of the OdometryProcessor
 *
 */
class Tester : public OdometryProcessor
{
  public:
    Tester()
        : OdometryProcessor(WHEEL_CIRCUMFERENCE, WHEEL_BASE, GEAR_RATIO, ROLLOVER, RIGHT_INCREASE,
                            LEFT_INCREASE)
    {
    }
    /**
     * @brief Wrapper to call the protected degree calculations on the processor
     *
     */
    void callDegreeCalculation()
    {
        this->calculateDegreesTraveledInFrame(Motor::LEFT);
        this->calculateDegreesTraveledInFrame(Motor::RIGHT);
    }

    /**
     * @brief Wrapper to call the protected meter calculations on the processor
     *
     */
    void callMeterCalculation()
    {
        this->calculateMetersMotorTraveledInFrame(Motor::LEFT);
        this->calculateMetersMotorTraveledInFrame(Motor::RIGHT);
    }

    /**
     * @brief Wrrapper to call protected frame distance function
     *
     */
    void callFrameDistanceCalculation() { this->calculateFrameDistance(); }

    /**
     * @brief Wrapper to call the protected theta calculation function
     *
     */
    void callThetaCalculation() { this->calculateTheta(); }

    /**
     * @brief Pass in enough readings to the processor for it to be settled and ready for
     * calculations
     *
     * @param leftValue left wheel reading
     * @param rightValue right wheel reading
     */
    void settleReadings(float leftValue, float rightValue)
    {
        for (int i = 0; i < SETTLE_READINGS; i++)
        {
            this->updateCurrentValue(Motor::LEFT, leftValue);
            this->updateCurrentValue(Motor::RIGHT, rightValue);
            // Update current time by 1 second
            this->startTime = 1000 * (i + 1);
            this->updateTimestamp(this->startTime); // Each reading happens one second apart
            this->processData();
        }
    }

    /**
     * @brief Drive the system one full encoder rotation
     *
     * @note Does not keep wheels moving at same rate
     */
    void driveFullEncoderRotation()
    {
        float leftEncoderReading = 120;
        float rightEncoderReading = 300;
        this->settleReadings(leftEncoderReading, rightEncoderReading);

        this->updateCurrentValue(Motor::LEFT, 350);  // -130
        this->updateCurrentValue(Motor::RIGHT, 100); // 160
        this->startTime += 1000;
        this->updateTimestamp(this->startTime); // Each reading happens one second apart

        this->processData();

        this->updateCurrentValue(Motor::LEFT, 260);  // -90
        this->updateCurrentValue(Motor::RIGHT, 190); // 90
        this->startTime += 1000;
        this->updateTimestamp(this->startTime); // Each reading happens one second apart
        this->processData();

        this->updateCurrentValue(Motor::LEFT, 160);  // -100
        this->updateCurrentValue(Motor::RIGHT, 290); // 100
        this->startTime += 1000;
        this->updateTimestamp(this->startTime); // Each reading happens one second apart
        this->processData();

        this->updateCurrentValue(Motor::LEFT, 120);  // 40
        this->updateCurrentValue(Motor::RIGHT, 300); // 10
        this->startTime += 1000;
        this->updateTimestamp(this->startTime); // Each reading happens one second apart
        this->processData();
    }

    /**
     * @brief Drive the system one full encoder rotation forward while keeping system perfectly
     * straight
     *
     */
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

// Test that system is updating values correctly
TEST(FrameTests, UpdateReadings)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 50;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

    auto currentLeftReading = processor.getCurrentReading(Motor::LEFT);
    auto currentRightReading = processor.getCurrentReading(Motor::RIGHT);

    ASSERT_EQ(leftEncoderReading, currentLeftReading);
    ASSERT_EQ(rightEncoderReading, currentRightReading);
}

// Test that system stabilizes
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

// Test that system calculates correct delta degrees for a single frame
TEST(FrameTests, DeltaDegrees)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 50;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

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

// Test that the system can handle encoder rollover and rollunder in a single frame
TEST(FrameTests, RollOver)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 300;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

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

// Check that system calculates correct velocity for a given frame
TEST(FrameTests, Velocity)
{
    auto processor = Tester();

    float leftEncoderReading = 120;
    float rightEncoderReading = 50;
    processor.settleReadings(leftEncoderReading, rightEncoderReading);

    leftEncoderReading = 90;
    rightEncoderReading = 80;

    processor.updateCurrentValue(Motor::LEFT, leftEncoderReading);
    processor.updateCurrentValue(Motor::RIGHT, rightEncoderReading);

    processor.updateTimestamp(processor.startTime + 500); // Each reading happens 0.5 second apart
    processor.processData();

    auto distanceInFrame = processor.getDistance().frameDistance;
    auto deltaAngle = processor.getPosition().theta;
    auto calculatedVelocity = processor.getVelocity();
    auto deltaTime = processor.getDeltaTime();

    ASSERT_EQ(deltaTime / 1000.0f, 0.5);

    ASSERT_EQ(2 * distanceInFrame, calculatedVelocity.linearX); // Meters / sec

    ASSERT_EQ(2 * deltaAngle, calculatedVelocity.angularZ); // Rad / sec
}

// Check system calculates correct distance moved from one full rotation on encoders
TEST(TotalTests, DeltaMetersOneEncoderRotation)
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

// Check that system handles total angle change of system
TEST(TotalTests, AngleTest)
{
    auto processor = Tester();

    processor.driveFullEncoderRotation();

    auto position = processor.getPosition();

    ASSERT_NEAR(0.0, position.theta, 0.001);
}

// Check that system calculates correct x, y position of system from start point
TEST(TotalTests, PositionTest)
{
    auto processor = Tester();

    processor.driveStraightOneEncoderRotation();

    auto totalDegreesLeft = processor.getTotalDegreesTraveled(Motor::LEFT);
    auto totalDegreesRight = processor.getTotalDegreesTraveled(Motor::RIGHT);

    ASSERT_EQ(-360.0, totalDegreesLeft);
    ASSERT_EQ(360.0, totalDegreesRight);

    auto position = processor.getPosition();

    float correctXPosition = WHEEL_CIRCUMFERENCE / GEAR_RATIO;

    ASSERT_NEAR(0, position.y, 0.01);
    ASSERT_NEAR(correctXPosition, position.x, 0.01);
}

// TODO: clp make this auto run on make -jn call
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}