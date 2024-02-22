#include "encoder_to_odom/odometry.h"

OdometryProcessor::OdometryProcessor(float wheelCircumference, float wheelBase, float gearRatio,
                                     float rolloverThreshold, bool rightIncrease, bool leftIncrease)
    : wheelCircumference(wheelCircumference), wheelBase(wheelBase), gearRatio(gearRatio),
      rolloverThreshold(rolloverThreshold), rightIncrease(rightIncrease), leftIncrease(leftIncrease)
{
}

void OdometryProcessor::updateCurrentValue(float value, Motor motor)
{

    // TODO: sync method that isnt just two bools

    // Update last reading with current reading in map
    this->lastReadings[motor] = this->currentReadings[motor];

    // Update current reading map with value from sensor
    this->currentReadings[motor] = value;

    // if (motor == Motor::LEFT)
    // {
    //     this->currentLeftReading = value;
    //     this->leftSync = true;
    // }
    // else if (motor == Motor::RIGHT)
    // {
    //     this->currentRightReading = value;
    //     this->rightSync = true;
    // }
}

float OdometryProcessor::calculateDeltaDegrees(float currentDegreeReading, float lastDegreeReading)
{
    // float deltaDegrees = 0.0;
    // if (currentDegreeReading != lastDegreeReading)
    // {
    float currentPreviousDelta = currentDegreeReading - lastDegreeReading;
    // float previousCurrentDelta = lastDegreeReading - currentDegreeReading;

    if (currentPreviousDelta > this->rolloverThreshold)
    {
        currentPreviousDelta = 0 - (360.0 - currentPreviousDelta);
    }

    else if (currentPreviousDelta < -this->rolloverThreshold)
    {
        currentPreviousDelta = 360.0 + currentPreviousDelta;
    }

    return currentPreviousDelta;
    // }
    // If current is same as last, return no delta
    // return 0;
}

// TODO: clp this is very  not dry one might even say this code is wet
void OdometryProcessor::calculateDegreesTraveledInFrame(Motor motor)
{

    // Get last and current reading copy
    auto currentReading = this->currentReadings[motor];
    auto lastReading = this->lastReadings[motor];

    // // Update last reading to be this frames current for next process loop
    // this->currentReadings[motor] = lastReading;

    float deltaDegrees = calculateDeltaDegrees(currentReading, lastReading);

    this->totalDegreesTraveled[motor] += deltaDegrees;

    // // Early return for n frames until system has stabilized
    // if (this->stablizationAmount > 0)
    // {
    //     this->stablizationAmount--;
    //     return 0.0;
    // }

    // if (motor == Motor::LEFT)
    // {
    //     totalLeftDegreesTraveled += deltaDegrees;
    // }
    // else if (motor == Motor::RIGHT)
    // {
    //     totalRightDegreesTraveled +=
    //         deltaDegrees; // TODO: clp negative because 500 install has right encoder forward as
    //         -
    // }

    // std::cout << "Total Left Degrees: " << totalLeftDegreesTraveled
    //           << " Total Right Degrees: " << totalRightDegreesTraveled << std::endl;

    // return deltaDegrees;
}

float OdometryProcessor::getTotalDegreesTraveled(Motor motor)
{
    return this->totalDegreesTraveled[motor];
}

void OdometryProcessor::calculateMetersMotorTraveledInFrame(Motor motor) // Per frame
{
    this->calculateDegreesTraveledInFrame(motor);

    auto totalDegrees = this->getTotalDegreesTraveled(motor);

    // Handle motors that forward is not increasing value changes
    if (!this->leftIncrease && motor == Motor::LEFT)
    {
        totalDegrees = -totalDegrees;
    }
    if (!this->rightIncrease && motor == Motor::RIGHT)
    {
        totalDegrees = -totalDegrees;
    }

    // Find number of encoder rotations based on wheel rotations
    float encoderRotations = totalDegrees / THREE_SIXTY;
    // Convert encoder rotations to wheel rotations
    float rotations = encoderRotations / this->gearRatio;

    // Convert wheel rotations to meters traveled
    float metersTraveled = rotations * this->wheelCircumference;

    this->metersTraveledInFrame[motor] = metersTraveled;
    this->totalMetersTraveled[motor] += metersTraveled;
    // return metersTraveled;
}

void OdometryProcessor::calculateFrameDistance()
{

    auto leftDistance = this->metersTraveledInFrame[Motor::LEFT];
    auto rightDistance = this->metersTraveledInFrame[Motor::RIGHT];

    this->distance.frameDistance = (rightDistance + leftDistance) / 2.0;

    this->distance.totalDistance += this->distance.frameDistance;

    // float deltaTime =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(end -
    //     this->linearStartTime).count() / 1000.0f;

    // int deltaSeconds = this->currentSec - this->lastSec;
    // int deltaNano = 0;

    // This assumes the delta time is never greater than 1 second
    // if (deltaSeconds > 0)
    // {
    //     // Seconds dont match we went up a second so nano delta is remainder of old plus new
    //     deltaNano += (1000000000 - this->lastNano); // Nano seconds remaing from last time
    //     deltaNano += this->currentNano;
    // }
    // else
    // {

    // deltaNano = this->currentNano - this->lastNano;
    // }

    // std::cout << "DS: " << deltaSeconds << " DN: " << deltaNano << std::endl;

    // float millisecondDelta = (deltaNano / 1000000) + (deltaSeconds * 1000);

    // float deltaTime = millisecondDelta / 1000.0f;
    // this->deltaTimeCapture = deltaTime;

    // if (this->stablizationAmount > 0)
    // {
    //     // this->stablizationAmount--;
    //     // return 0.0;
    //     return;
    // }

    // if (deltaTime < 0.01)
    // {
    //     deltaTime = 0.015;
    // }

    // M/S velocity relative to the robot frame??
    // TODO: clp make sure this ^ is true

    // if (deltaTime > 0.005)
    // {
    // float velocity = this->frameDistance / (deltaTime); // should be deltaTime
    // if (std::abs(velocity) < 5.0)
    // {
    //     if (std::abs(velocity) > std::abs(this->maxVelocity))
    //     {
    //         std::cout << "---------------------------------------------------------------------"
    //                      "----------------------------------"
    //                   << std::endl;
    //         this->maxVelocity = velocity;
    //     }

    //     std::cout << "DT: " << deltaTime << " distance: " << this->frameDistance
    //               << " M/S: " << velocity << " MAX: " << this->maxVelocity
    //               << " Total Dist: " << this->totalDistance << std::endl;
    //     this->linearVelocity = velocity;
    // }
    // }
    // this->start = std::chrono::system_clock::now();
    // this->linearStartTime = std::chrono::system_clock::now();
}

// Radians
void OdometryProcessor::calculateTheta()
{
    // auto end = std::chrono::system_clock::now();

    // float deltaTime =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(end - this->angularStartTime)
    //         .count() /
    //     1000.0f;

    // int deltaSeconds = this->currentSec - this->lastSec;
    // int deltaNano = 0;
    // This assumes the delta time is never greater than 1 second
    // if (deltaSeconds > 0)
    // {
    //     // Seconds dont match we went up a second so nano delta is remainder of old plus new
    //     deltaNano += (1000000000 - this->lastNano); // Nano seconds remaing from last time
    //     deltaNano += this->currentNano;
    // }
    // else
    // {
    // deltaNano = this->currentNano - this->lastNano;
    // }

    // float millisecondDelta = (deltaNano / 1000000) + (deltaSeconds * 1000);

    // float deltaTime = millisecondDelta / 1000.0f;

    auto rightDistance = this->metersTraveledInFrame[Motor::RIGHT];
    auto leftDistance = this->metersTraveledInFrame[Motor::LEFT];

    float difference = rightDistance - leftDistance;
    float angle = asinf(difference / this->wheelBase); // Radians

    // if (deltaTime < 0.01)
    // {
    //     deltaTime = 0.015;
    // }

    // float velocity = angle / (deltaTime);

    // this->angularVelocity = velocity;

    // std::cout << "Angular M/S: " << this->angularVelocity << std::endl;
    // if (!firstRun)
    // {
    this->currentPosition.theta += (angle);
    if (this->currentPosition.theta > PI)
    {
        this->currentPosition.theta -= 2.0 * PI;
    }
    else if (this->currentPosition.theta < -PI)
    {
        this->currentPosition.theta += 2.0 * PI;
    }
    // }
    // return angle;

    // std::cout << "ld: " << leftDistance << " rd: " << rightDistance << " diff: " << difference
    //   << std::endl;
}

// bool OdometryProcessor::sync()
// {
//     if (this->leftSync && this->rightSync)
//     {
//         this->leftSync = false;
//         this->rightSync = false;
//         return true;
//     }
//     return false;
// }

void OdometryProcessor::calculateDistanceMovedX()
{
    // if (!firstRun)
    // {

    float distanceMoved = cosf(this->currentPosition.theta) * this->distance.frameDistance;

    this->currentPosition.x += distanceMoved;
    // }
}

void OdometryProcessor::calculateDistanceMovedY()
{
    // if (!firstRun)

    // {
    float distanceMoved = sinf(this->currentPosition.theta) * this->distance.frameDistance;
    this->currentPosition.x += distanceMoved;

    // std::cout << "Frame Dist: " << this->frameDistance
    //           << " total Pose: " << this->currentTotalPoseTheta
    //           << "Distance Moved: " << distanceMoved << "totalDistY: " << totalDistanceY
    //           << std::endl;
    // }
    // this->firstRun = false;
}

Position OdometryProcessor::getPosition() { return this->currentPosition; }

// Velocity OdometryProcessor::getVelocity()
// {
//     Velocity velocity = {this->linearVelocity, this->angularVelocity};
//     return velocity;
// }

bool OdometryProcessor::settled()
{
    if (this->stablizationAmount > 0)
    {
        this->stablizationAmount--;
        return false;
    }
    return true;
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

    // // Make all internal calls
    // // if (sync())
    // // {

    // // std::cout << "Synced" << std::endl;
    // // Get distance traveled for each wheel
    // // Per frame
    // float currentLeftDegreesTraveled = -calculateDegreesTraveledInFrame(Motor::LEFT);
    // float currentRightDegreesTraveled = calculateDegreesTraveledInFrame(Motor::RIGHT);

    // // Per Frame
    // // std::cout << "Left ";
    // float leftMetersTraveled = calculateMetersTraveled(currentLeftDegreesTraveled);

    // // std::cout << "Right ";
    // float rightMetersTraveled = calculateMetersTraveled(currentRightDegreesTraveled);

    // // this->totalLeftMeterTraveled += leftMetersTraveled;
    // // this->totalRightMetersTraveled += rightMetersTraveled;

    // std::cout << "LM: " << this->totalLeftMeterTraveled << " RM: " <<
    // this->totalRightMetersTraveled
    //           << std::endl;

    // // Pick which motors to use for distance default is left and right motor

    // calculateFrameDistance();

    // // this->linearStartTime = std::chrono::system_clock::now();
    // // this->angularStartTime = std::chrono::system_clock::now();

    // calculateHeadingAngleDelta();

    // calculateDistanceMovedX();
    // calculateDistanceMovedY();
    // }
}
