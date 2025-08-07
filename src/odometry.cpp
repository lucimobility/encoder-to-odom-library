/**
 * @file odometry.cpp
 * @brief File to implement all odom math and encoder convertions
 * @date 2024-03-06
 *
 * @copyright Copyright (c) 2024 LUCI Mobility, Inc. All Rights Reserved.
 *
 */

#include "odometry.h"

static constexpr float SMALL_ANGLE_THRESHOLD_RAD = 0.01f; // ~0.57 degrees

OdometryProcessor::OdometryProcessor(float wheelCircumference, float wheelBase, float gearRatio,
                                     float rolloverThreshold, bool rightMotorForwardIncreases,
                                     bool leftMotorForwardIncreases)
    : wheelCircumference(wheelCircumference), wheelBase(wheelBase), gearRatio(gearRatio),
      rolloverThreshold(rolloverThreshold), rightMotorForwardIncreases(rightMotorForwardIncreases),
      leftMotorForwardIncreases(leftMotorForwardIncreases)
{
    // Initialize the motor readings
    this->currentEncoderAngles[Motor::LEFT] = 0.0;
    this->currentEncoderAngles[Motor::RIGHT] = 0.0;
}

void OdometryProcessor::updateEncoderReading(Motor motor, float angleInDegrees)
{
    // Store current reading as previous for next frame's delta calculation
    this->previousEncoderAngles[motor] = this->currentEncoderAngles[motor];

    // Update current reading map with value from sensor
    this->currentEncoderAngles[motor] = angleInDegrees;
}

void OdometryProcessor::updateTimestamp(uint16_t newTimestamp)
{
    this->deltaTime = newTimestamp - this->timestamp;
    this->timestamp = newTimestamp;
}

bool OdometryProcessor::settled()
{
    if (this->stabilizationAmount > 0)
    {
        this->stabilizationAmount--;
        return false;
    }
    return true;
}

// Calculations
float OdometryProcessor::calculateAngleChange(float currentDegreeReading, float lastDegreeReading)
{
    float angleDifference = currentDegreeReading - lastDegreeReading;

    // Is the change in angle large enough to be a rollover
    if (angleDifference > this->rolloverThreshold)
    {
        angleDifference = 0 - (THREE_SIXTY - angleDifference);
    }
    else if (angleDifference < -this->rolloverThreshold)
    {
        angleDifference = THREE_SIXTY + angleDifference;
    }
    return angleDifference;
}

void OdometryProcessor::calculateDegreesTraveledInFrame(Motor motor)
{
    float currentReading = this->getCurrentEncoderAngles(motor);
    float lastReading = this->getPreviousEncoderAngles(motor);

    float angleChange = calculateAngleChange(currentReading, lastReading);

    // Accumulate total rotation for this motor since startup
    this->totalDegreesTraveled[motor] += angleChange;
    this->degreesTraveledInFrame[motor] = angleChange;
}

void OdometryProcessor::calculateMetersTraveledInFrame(Motor motor)
{
    this->calculateDegreesTraveledInFrame(motor);

    auto angleChange = this->getDegreesTraveledInFrame(motor);

    // Invert angle delta if this motor decreases when moving forward
    if (!this->leftMotorForwardIncreases && motor == Motor::LEFT)
    {
        angleChange = -angleChange;
    }
    if (!this->rightMotorForwardIncreases && motor == Motor::RIGHT)
    {
        angleChange = -angleChange;
    }

    // Find number of encoder rotations based on wheel rotations
    float encoderRotations = angleChange / THREE_SIXTY;
    // Convert encoder rotations to wheel rotations
    float wheelRotations = encoderRotations / this->gearRatio;

    // Convert wheel rotations to meters traveled
    float metersTraveled = wheelRotations * this->wheelCircumference;

    // Accumulate total distance traveled for this motor since startup
    this->totalMetersTraveled[motor] += metersTraveled;
    this->metersTraveledInFrame[motor] = metersTraveled;
}

void OdometryProcessor::calculateFrameDistance()
{
    auto leftDistance = this->getMetersTraveledInFrame(Motor::LEFT);
    auto rightDistance = this->getMetersTraveledInFrame(Motor::RIGHT);

    // Calculate the average distance traveled by both motors in this frame
    this->distance.frameDistance = (rightDistance + leftDistance) / 2.0;

    // Avoid division by zero for deltaTime
    if (this->getDeltaTime() > 0)
    {
        this->velocity.linearX = this->distance.frameDistance / (this->getDeltaTime() / 1000.0f);
    }
    else
    {
        this->velocity.linearX = 0.0f;
    }

    // Accumulate total distance traveled since startup
    this->distance.totalDistance += this->distance.frameDistance;
}

void OdometryProcessor::calculateTheta()
{
    auto rightDistance = this->metersTraveledInFrame[Motor::RIGHT];
    auto leftDistance = this->metersTraveledInFrame[Motor::LEFT];

    // Calculate the change in orientation using differential drive kinematics
    // For a differential drive robot: dTheta = (rightDistance - leftDistance) / wheelBase
    float deltaTheta = (rightDistance - leftDistance) / this->wheelBase;

    // Calculate angular velocity (rad/s)
    // Avoid division by zero for deltaTime
    if (this->getDeltaTime() > 0)
    {
        this->velocity.angularZ = deltaTheta / (this->getDeltaTime() / 1000.0f);
    }
    else
    {
        this->velocity.angularZ = 0.0f;
    }

    // Accumulate the change in orientation
    this->currentPosition.theta += deltaTheta;

    // Normalize theta to [-π, π] range using modulo arithmetic
    this->currentPosition.theta = normalizeAngle(this->currentPosition.theta);
}

float OdometryProcessor::normalizeAngle(float angle)
{
    // Use fmod for floating point modulo operation
    angle = fmod(angle + PI, 2.0f * PI);
    if (angle < 0)
    {
        angle += 2.0f * PI;
    }
    return angle - PI;
}

void OdometryProcessor::calculatePositionChange()
{
    auto rightDistance = this->metersTraveledInFrame[Motor::RIGHT];
    auto leftDistance = this->metersTraveledInFrame[Motor::LEFT];

    float deltaTheta = (rightDistance - leftDistance) / this->wheelBase;

    float deltaX, deltaY;

    if (fabs(deltaTheta) < SMALL_ANGLE_THRESHOLD_RAD)
    {
        // Small angle approximation (~0.57 degrees) - use midpoint theta
        float midTheta = this->currentPosition.theta - deltaTheta / 2.0f;
        deltaX = this->distance.frameDistance * cosf(midTheta);
        deltaY = this->distance.frameDistance * sinf(midTheta);
    }
    else
    {
        // Large angle - use exact arc geometry
        float radius = this->distance.frameDistance / deltaTheta;
        float prevTheta = this->currentPosition.theta - deltaTheta;

        deltaX = radius * (sinf(this->currentPosition.theta) - sinf(prevTheta));
        deltaY = radius * (cosf(prevTheta) - cosf(this->currentPosition.theta));
    }

    // Update position
    this->currentPosition.x += deltaX;
    this->currentPosition.y += deltaY;
}

void OdometryProcessor::processData()
{
    if (settled())
    {
        this->calculateMetersTraveledInFrame(Motor::LEFT);
        this->calculateMetersTraveledInFrame(Motor::RIGHT);

        this->calculateFrameDistance();
        this->calculateTheta();

        this->calculatePositionChange();
    }
}

void OdometryProcessor::reset()
{
    // Reset all readings
    this->currentEncoderAngles[Motor::LEFT] = 0.0;
    this->currentEncoderAngles[Motor::RIGHT] = 0.0;

    this->previousEncoderAngles[Motor::LEFT] = 0.0;
    this->previousEncoderAngles[Motor::RIGHT] = 0.0;

    this->velocity.linearX = 0.0;
    this->velocity.angularZ = 0.0;

    this->resetTotalDegreesTraveled();
    this->resetTotalMetersTraveled();

    this->resetDistance();
    this->resetPosition();

    // Reset timestamp and delta time
    this->timestamp = 0;
    this->deltaTime = 0;

    // Reset the stabilization counter
    this->stabilizationAmount = STABILIZATION_FRAMES;
}

void OdometryProcessor::resetDistance()
{
    this->distance.frameDistance = 0.0;
    this->distance.totalDistance = 0.0;
}

void OdometryProcessor::resetPosition()
{
    this->currentPosition.x = 0.0;
    this->currentPosition.y = 0.0;
    this->currentPosition.theta = 0.0;
}

void OdometryProcessor::resetTotalDegreesTraveled()
{
    this->totalDegreesTraveled[Motor::LEFT] = 0.0;
    this->totalDegreesTraveled[Motor::RIGHT] = 0.0;
}

void OdometryProcessor::resetTotalMetersTraveled()
{
    this->totalMetersTraveled[Motor::LEFT] = 0.0;
    this->totalMetersTraveled[Motor::RIGHT] = 0.0;
}

// Getters
Position OdometryProcessor::getPosition() const { return this->currentPosition; }

Velocity OdometryProcessor::getVelocity() const { return this->velocity; }

Distance OdometryProcessor::getDistance() const { return this->distance; }

float OdometryProcessor::getTotalDegreesTraveled(Motor motor)
{
    return this->totalDegreesTraveled[motor];
}

float OdometryProcessor::getTotalMetersTraveled(Motor motor)
{
    return this->totalMetersTraveled[motor];
}

float OdometryProcessor::getDegreesTraveledInFrame(Motor motor)
{
    return this->degreesTraveledInFrame[motor];
}

float OdometryProcessor::getMetersTraveledInFrame(Motor motor)
{
    return this->metersTraveledInFrame[motor];
}

float OdometryProcessor::getCurrentEncoderAngles(Motor motor)
{
    return this->currentEncoderAngles[motor];
}

float OdometryProcessor::getPreviousEncoderAngles(Motor motor)
{
    return this->previousEncoderAngles[motor];
}

int OdometryProcessor::getDeltaTime() { return this->deltaTime; }
