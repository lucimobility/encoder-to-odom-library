/**
 * @file odometry.cpp
 * @brief File to implement all odom math and encoder convertions
 * @date 2024-03-06
 *
 * @copyright Copyright (c) 2024 LUCI Mobility, Inc. All Rights Reserved.
 *
 */

#include "odometry.h"

OdometryProcessor::OdometryProcessor(float wheelCircumference, float wheelBase, float gearRatio,
                                     float rolloverThreshold, bool rightIncrease, bool leftIncrease)
    : wheelCircumference(wheelCircumference), wheelBase(wheelBase), gearRatio(gearRatio),
      rolloverThreshold(rolloverThreshold), rightIncrease(rightIncrease), leftIncrease(leftIncrease)
{
    // Initialize the motor readings
    this->currentReadings[Motor::LEFT] = 0.0;
    this->currentReadings[Motor::RIGHT] = 0.0;
}
// Setters
void OdometryProcessor::updateCurrentValue(Motor motor, float value)
{
    // Update last reading with current reading in map
    this->lastReadings[motor] = this->currentReadings[motor];

    // Update current reading map with value from sensor
    this->currentReadings[motor] = value;
}

void OdometryProcessor::updateTimestamp(uint16_t timestamp)
{
    // Calculate delta time
    this->deltaTime = timestamp - this->timestamp;
    // Update reading for next frame delta time
    this->timestamp = timestamp;
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
float OdometryProcessor::calculateDeltaDegrees(float currentDegreeReading, float lastDegreeReading)
{
    float currentPreviousDelta = currentDegreeReading - lastDegreeReading;

    // Is the change in angle large enough to be a rollover
    if (currentPreviousDelta > this->rolloverThreshold)
    {
        currentPreviousDelta = 0 - (THREE_SIXTY - currentPreviousDelta);
    }

    // Is the change in angle in negative a direction enough to be a rollunder
    else if (currentPreviousDelta < -this->rolloverThreshold)
    {
        currentPreviousDelta = THREE_SIXTY + currentPreviousDelta;
    }
    return currentPreviousDelta;
}

void OdometryProcessor::calculateDegreesTraveledInFrame(Motor motor)
{

    // Get last and current reading copy
    auto currentReading = this->getCurrentReading(motor);
    auto lastReading = this->getLastReading(motor);

    // calculate the delta degrees
    float deltaDegrees = calculateDeltaDegrees(currentReading, lastReading);

    // Update motors entry values
    this->totalDegreesTraveled[motor] += deltaDegrees;
    this->degreesTraveledInFrame[motor] = deltaDegrees;
}

void OdometryProcessor::calculateMetersMotorTraveledInFrame(Motor motor) // Per frame
{
    this->calculateDegreesTraveledInFrame(motor);

    auto deltaDegrees = this->getDegreesTraveledInFrame(motor);

    // Handle motors that forward is not increasing value changes
    if (!this->leftIncrease && motor == Motor::LEFT)
    {
        deltaDegrees = -deltaDegrees;
    }
    if (!this->rightIncrease && motor == Motor::RIGHT)
    {
        deltaDegrees = -deltaDegrees;
    }

    // Find number of encoder rotations based on wheel rotations
    float encoderRotations = deltaDegrees / THREE_SIXTY;
    // Convert encoder rotations to wheel rotations
    float rotations = encoderRotations / this->gearRatio;

    // Convert wheel rotations to meters traveled in this frame
    float metersTraveled = rotations * this->wheelCircumference;

    this->metersTraveledInFrame[motor] = metersTraveled;
    this->totalMetersTraveled[motor] += metersTraveled;
}

void OdometryProcessor::calculateFrameDistance()
{

    auto leftDistance = this->getMetersTraveledInFrame(Motor::LEFT);
    auto rightDistance = this->getMetersTraveledInFrame(Motor::RIGHT);

    this->distance.frameDistance = (rightDistance + leftDistance) / 2.0;

    std::cout << "DT (milli): " << this->getDeltaTime()
              << " DT (sec): " << this->getDeltaTime() / 1000.0f << std::endl;

    this->velocity.linearX = this->distance.frameDistance / (this->getDeltaTime() / 1000.0f);

    this->distance.totalDistance += this->distance.frameDistance;
}

// Radians
void OdometryProcessor::calculateTheta()
{

    auto rightDistance = this->metersTraveledInFrame[Motor::RIGHT];
    auto leftDistance = this->metersTraveledInFrame[Motor::LEFT];

    // Delta between two motors traveled
    float difference = rightDistance - leftDistance;

    float angle = asinf(difference / this->wheelBase); // Radians
    // Radians / sec
    this->velocity.angularZ = angle / (this->getDeltaTime() / 1000.0f);

    this->currentPosition.theta += (angle);
    // Restrain theta to a single 180
    if (this->currentPosition.theta > PI)
    {
        this->currentPosition.theta -= 2.0 * PI;
    }
    else if (this->currentPosition.theta < -PI)
    {
        this->currentPosition.theta += 2.0 * PI;
    }
}

void OdometryProcessor::calculateDistanceMovedX()
{
    float distanceMoved = cosf(this->currentPosition.theta) * this->distance.frameDistance;
    this->currentPosition.x += distanceMoved;
}

void OdometryProcessor::calculateDistanceMovedY()
{
    float distanceMoved = sinf(this->currentPosition.theta) * this->distance.frameDistance;
    this->currentPosition.y += distanceMoved;
}

void OdometryProcessor::processData()
{

    if (settled())
    {
        this->calculateMetersMotorTraveledInFrame(Motor::LEFT);
        this->calculateMetersMotorTraveledInFrame(Motor::RIGHT);

        this->calculateFrameDistance();
        this->calculateTheta();

        this->calculateDistanceMovedX();
        this->calculateDistanceMovedY();
    }
}

void OdometryProcessor::reset()
{
    // Reset all readings
    this->currentReadings[Motor::LEFT] = 0.0;
    this->currentReadings[Motor::RIGHT] = 0.0;

    this->lastReadings[Motor::LEFT] = 0.0;
    this->lastReadings[Motor::RIGHT] = 0.0;

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
    this->stabilizationAmount = SETTLE_READINGS;
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
float OdometryProcessor::getCurrentReading(Motor motor) { return this->currentReadings[motor]; }

float OdometryProcessor::getLastReading(Motor motor) { return this->lastReadings[motor]; }

Distance OdometryProcessor::getDistance() { return this->distance; }

int OdometryProcessor::getDeltaTime() { return this->deltaTime; }

Velocity OdometryProcessor::getVelocity() { return this->velocity; }

Position OdometryProcessor::getPosition() { return this->currentPosition; }

float OdometryProcessor::getTotalDegreesTraveled(Motor motor)
{
    return this->totalDegreesTraveled[motor];
}

float OdometryProcessor::getDegreesTraveledInFrame(Motor motor)
{
    return this->degreesTraveledInFrame[motor];
}

float OdometryProcessor::getTotalMetersTraveled(Motor motor)
{
    return this->totalMetersTraveled[motor];
}

float OdometryProcessor::getMetersTraveledInFrame(Motor motor)
{
    return this->metersTraveledInFrame[motor];
}
