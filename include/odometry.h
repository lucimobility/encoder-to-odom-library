/**
 * @file odometry.h
 * @brief Class and helper types that allow for the conversion of encoder angle readings to odometry
 * values
 * @date 2024-03-06
 *
 * @copyright Copyright (c) 2024 LUCI Mobility, Inc. All Rights Reserved.
 */

#pragma once
#include <chrono>
#include <iostream>
#include <map>
#include <math.h>

/// @brief  General reusable values
constexpr float PI = 3.14159265;
constexpr float THREE_SIXTY = 360.0;
constexpr int SETTLE_READINGS = 3;

/**
 * @brief Enum to determine which motor an encoder is attached to
 *
 * @note If a system has more then right and left motors being tracked by encoders add them here
 *
 */
enum class Motor
{
    LEFT,
    RIGHT
};

/**
 * @brief Position data of the object from starting point
 *
 */
struct Position
{
    float x;     /// X position of robot following standard right hand rule (forward-back distance)
    float y;     /// Y position of robot following standard right hand rule (side-to-side distance)
    float theta; /// Theta from start position in radians (rotation about z axis)
};

/**
 * @brief Velocity of robot at given time
 *
 */
struct Velocity
{
    float linearX;  /// Linear forward-back velocity
    float angularZ; /// Turning velocity
};

/**
 * @brief Distance struct for frame distance and total distance from startup
 *
 */
struct Distance
{
    float frameDistance; /// Distance that the system moved (in meters) in the single frame
    float totalDistance; /// Distance that the system has moved (in meters) since being started
};

class OdometryProcessor
{
  public:
    /**
     * @brief Construct a new Odometry Processor object
     *
     * @param wheelCircumference The circumference of the wheels (this assumes both drive wheels are
     * the same size) (meters)
     * @param wheelBase The distance between the centerpoint of both drive wheels (meters)
     * @param gearRatio The number of encoder degrees read per 1 degree of wheel travel (float)
     * @param rolloverThreshold The number of degrees traveled in a single frame by the encoder to
     * trigger a rollover event (int)
     * @param rightIncrease If the right motor increases in values as the system moves forward
     * (bool)
     * @param leftIncrease If the left motor increases in values as the system moves forward (bool)
     */
    OdometryProcessor(float wheelCircumference, float wheelBase, float gearRatio,
                      float rolloverThreshold, bool rightIncrease = true, bool leftIncrease = true);

    /**
     * @brief Update with the latest encoder readings
     *
     * @param value Encoder angle reading
     */
    void updateCurrentValue(Motor motor, float value);

    /**
     * @brief Calculate the total distance the robot moved in the x axis
     *
     */
    void calculateDistanceMovedX();

    /**
     * @brief Calculate the total distance the robot moved in the y axis
     *
     */
    void calculateDistanceMovedY();

    /**
     * @brief Run all private calls to process new data frame
     *
     */
    void processData();

    /**
     * @brief Reset the odometry processor to initial state
     *
     */
    void reset();

    /**
     * @brief Get the Position object
     *
     * @return Position (x,y,theta) or robot in odom coordinate frame
     */
    Position getPosition();

    /**
     * @brief Get the Velocity object
     *
     * @return Velocity (linear m/s and angular rad/s)
     *
     * @note For velocity calculations the library assumes the encoder processor (arduino, stm,
     * odrive) is offering some form of consistent time stamp. See README for more details on this.
     */
    Velocity getVelocity();

    /**
     * @brief Get the Distance object
     *
     * @return Distance (meters traveled in last frame and meters traveled since boot up)
     */
    Distance getDistance();

    /**
     * @brief Get the Wheel Circumference object
     *
     * @return float wheel circumference in meters
     */
    float getWheelCircumference();

    /**
     * @brief Get the Wheel Base object
     *
     * @return float distance between the two drive wheels in meters
     */
    float getWheelBase();

    /**
     * @brief Get the Gear Ratio object
     *
     * @return float wheel to encoder ratio
     */
    float getGearRatio();

    /**
     * @brief Get the total degrees traveled of a single motor since powering up
     *
     * @param motor Which motor you want the degrees from
     * @return float degrees traveled by motor
     */
    float getTotalDegreesTraveled(Motor motor);

    /**
     * @brief Get the total meters traveled of a single motor since powering up
     *
     * @param motor Which motor you want meters from
     * @return float total meters traveled by motor
     */
    float getTotalMetersTraveled(Motor motor);

    /**
     * @brief Get the degrees traveled by a single motor in a single frame
     *
     * @param motor
     * @return float degrees traveled by motor in frame
     */
    float getDegreesTraveledInFrame(Motor motor);

    /**
     * @brief Get the meters traveled by a single motor in a single frame
     *
     * @param motor
     * @return float meters traveled by a motor in a single frame
     */
    float getMetersTraveledInFrame(Motor motor);

    /**
     * @brief Get the Current Reading object
     *
     * @param motor
     * @return float
     */
    float getCurrentReading(Motor motor);

    /**
     * @brief Get the Last Reading object
     *
     * @param motor
     * @return float
     */
    float getLastReading(Motor motor);

    /**
     * @brief Update the latest timestamp of received data
     *
     * @param timestamp
     */
    void updateTimestamp(uint16_t timestamp);

    /**
     * @brief Calculate delta time
     *
     * @return int delta of timestamp units since last frame
     */
    int getDeltaTime();

  protected:
    /**
     * @brief Calculates the degrees the encoder moved in one frame
     *
     * @param motor Which drive motor the current angle reading should be associated with
     */
    void calculateDegreesTraveledInFrame(Motor motor);

    /**
     * @brief Handle the rollover / rollunder of encoders (360->1), (1->360)
     *v
     * @param currentDegreeReading Current reading from the encoder
     * @param lastDegreeReading Last recorded reading from the encoder
     * @return float delta degree between last and current frame of the encoder
     */
    float calculateDeltaDegrees(float currentDegreeReading, float lastDegreeReading);

    /**
     * @brief Calculate the meters a single motor traveled in single frame
     *
     * @param motor
     */
    void calculateMetersMotorTraveledInFrame(Motor motor);

    /**
     * @brief Calculate the distance traveled of system in single frame
     *
     */
    void calculateFrameDistance();

    /**
     * @brief Calculate the total radians traveled since boot
     *
     */
    void calculateTheta();

    /**
     * @brief Evaluate if system has stabilized from first few readings at boot
     *
     * @return true system values are good to use
     * @return false system values are still stabilizing
     */
    bool settled();

    /// Circumference of robot wheels in meters
    float wheelCircumference = 0.0;

    /// Distance between wheel centers of robot in meters
    float wheelBase = 0.0;

    /// Number of rotations encoder makes per 1 wheel rotation
    float gearRatio = 1.0;

    /// Angle value that a delta change triggers a rollover
    /// Should be fine based on system max speed
    float rolloverThreshold = 100.0;

    /// The last processed Delta angle between encoder frames
    float previousLeftDegree = 0.0;
    float previousRightDegree = 0.0;

    /// Sync trackers to determine if new data has been produced by the encoders
    bool leftSync = false;
    bool rightSync = false;

    /// Position of system
    Position currentPosition = {0, 0, 0};
    /// Distance system has traveled
    Distance distance = {0, 0};
    /// Velocity of system at given frame
    Velocity velocity = {0, 0};

    /// Mappings for each motors individual recorded values
    std::map<Motor, float> currentReadings;
    std::map<Motor, float> lastReadings;
    std::map<Motor, float> totalDegreesTraveled;
    std::map<Motor, float> metersTraveledInFrame;
    std::map<Motor, float> totalMetersTraveled;
    std::map<Motor, float> degreesTraveledInFrame;

    /// Number of readings to throw out before considering the system stabilized
    int stabilizationAmount = SETTLE_READINGS;

    /// Which direction positive change in encoder values should be expected
    bool rightIncrease;
    bool leftIncrease;

    /// Timestamp of reading from edge device such as Arduino
    uint16_t timestamp = 0;
    int deltaTime = 0;
};