#include "encoder_to_odom/odometry.h"

OdometryProcessor::OdometryProcessor(float wheelCircumference, float wheelBase, float gearRatio,
                                     float rolloverThreshold, bool rightIncrease, bool leftIncrease)
    : wheelCircumference(wheelCircumference), wheelBase(wheelBase), gearRatio(gearRatio),
      rolloverThreshold(rolloverThreshold), rightIncrease(rightIncrease), leftIncrease(leftIncrease)
{
    this->currentReadings[Motor::LEFT] = 0.0;
    this->currentReadings[Motor::RIGHT] = 0.0;
}

float OdometryProcessor::getCurrentReading(Motor motor) { return this->currentReadings[motor]; }

float OdometryProcessor::getLastReading(Motor motor) { return this->lastReadings[motor]; }

void OdometryProcessor::updateCurrentValue(Motor motor, float value)
{

    // TODO: sync method that isnt just two bools

    // Update last reading with current reading in map
    this->lastReadings[motor] = this->currentReadings[motor];

    // Update current reading map with value from sensor
    this->currentReadings[motor] = value;
}

void OdometryProcessor::updateTimestamp(uint16_t timestamp)
{
    this->deltaTime = timestamp - this->timestamp;
    this->timestamp = timestamp;
}

float OdometryProcessor::calculateDeltaDegrees(float currentDegreeReading, float lastDegreeReading)
{
    float currentPreviousDelta = 0.0;
    if (currentDegreeReading != lastDegreeReading)
    {
        float currentPreviousDelta = currentDegreeReading - lastDegreeReading;

        if (currentPreviousDelta > this->rolloverThreshold)
        {
            currentPreviousDelta = 0 - (360.0 - currentPreviousDelta);
        }

        else if (currentPreviousDelta < -this->rolloverThreshold)
        {
            currentPreviousDelta = 360.0 + currentPreviousDelta;
        }
    }
    return currentPreviousDelta;
}

void OdometryProcessor::calculateDegreesTraveledInFrame(Motor motor)
{

    // Get last and current reading copy
    auto currentReading = this->getCurrentReading(motor);
    auto lastReading = this->getLastReading(motor);

    float deltaDegrees = calculateDeltaDegrees(currentReading, lastReading);

    this->totalDegreesTraveled[motor] += deltaDegrees;
    this->degreesTraveledInFrame[motor] = deltaDegrees;
}

float OdometryProcessor::getTotalDegreesTraveled(Motor motor)
{
    return this->totalDegreesTraveled[motor];
}

float OdometryProcessor::getDegreesTraveledInFrame(Motor motor)
{
    return this->degreesTraveledInFrame[motor];
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

float OdometryProcessor::getTotalMetersTraveled(Motor motor)
{
    return this->totalMetersTraveled[motor];
}

float OdometryProcessor::getMetersTraveledInFrame(Motor motor)
{
    return this->metersTraveledInFrame[motor];
}

void OdometryProcessor::calculateFrameDistance()
{

    auto leftDistance = this->getMetersTraveledInFrame(Motor::LEFT);
    auto rightDistance = this->getMetersTraveledInFrame(Motor::RIGHT);

    this->distance.frameDistance = (rightDistance + leftDistance) / 2.0;

    this->velocity.linearY = this->distance.frameDistance / (this->getDeltaTime() / 1000.0f);

    this->distance.totalDistance += this->distance.frameDistance;
}

Distance OdometryProcessor::getDistance() { return this->distance; }

// Radians
void OdometryProcessor::calculateTheta()
{

    auto rightDistance = this->metersTraveledInFrame[Motor::RIGHT];
    auto leftDistance = this->metersTraveledInFrame[Motor::LEFT];

    float difference = rightDistance - leftDistance;
    this->velocity.angularZ = difference / (this->getDeltaTime() / 1000.0f);

    float angle = asinf(difference / this->wheelBase); // Radians

    this->currentPosition.theta += (angle);
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

Position OdometryProcessor::getPosition() { return this->currentPosition; }

bool OdometryProcessor::settled()
{
    if (this->stablizationAmount > 0)
    {
        this->stablizationAmount--;
        return false;
    }
    return true;
}

uint16_t OdometryProcessor::getDeltaTime() { return this->deltaTime; }

Velocity OdometryProcessor::getVelocity() { return this->velocity; }

// void OdometryProcessor::calculateSystemVelocity()
// {
//     auto frameDistance = this->get;
//     auto frameTheta = this->getDegreesTraveledInFrame();

//     auto deltaTime = this->getDeltaTime();
// }

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

        // this->calculateSystemVelocity();
    }
}
