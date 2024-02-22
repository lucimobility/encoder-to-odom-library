#pragma once
#include <chrono>
#include <iostream>
#include <map>
#include <math.h>

// TODO: this vs consexpr float
#define PI 3.14159265
#define THREE_SIXTY 360.0

/**
 * @brief Enum to determin which motor an encoder is attached to
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
    float x;     /// X position of robot following standard right hand rule (side-to-side distance)
    float y;     /// Y position of robot following standard right hand rule (forward-back distance)
    float theta; /// Theta from start position in radians (rotation about z axis)
};

/**
 * @brief Velocity of robot at given time
 *
 */
struct Velocity
{
    float linearY;  /// Linear forward-back velocity
    float angularZ; /// Turning velocity
};

/**
 * @brief Distance struct for frame distance and total distance from startup
 *
 */
struct Distance
{
    float frameDistance;
    float totalDistance;
};

class OdometryProcessor
{
  public:
    /**
     * @brief Construct a new Odometry Processor object
     *
     * @param wheelCircumfrance Circumfrance in meters of robots wheels
     * @param wheelBase Distance between center of wheels in meters
     * @param gearRatio Number of encoder rotations per 1 wheel rotation, ex) an encoder mounted
     * @param rolloverThreshold Delta value that if seen indicates a rollover of the encoder
     * directly on the robots wheel this number would be 1.0
     */
    OdometryProcessor(float wheelCircumference, float wheelBase, float gearRatio,
                      float rolloverThreshold, bool rightIncrease = true, bool leftIncrease = true);

    /**
     * @brief Update the
     *
     * @param value
     */
    void updateCurrentValue(float value, Motor motor);

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

    // /**
    //  * @brief Check if the robot has produced new left and right motor readings
    //  *
    //  * @return true new values for both motor encoders are available
    //  * @return false new values for both motor encoders are NOT yet available
    //  */
    // bool sync();

    /**
     * @brief Run all private calls to process new data frame
     *
     */
    void processData();

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

    float getWheelCircumference();

    float getWheelBase();

    float getGearRatio();

    float getTotalDegreesTraveled(Motor motor);

    float getTotalMetersTraveled(Motor motor);

  private:
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

    void calculateMetersMotorTraveledInFrame(Motor motor);

    void calculateFrameDistance();

    void calculateTheta();

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

    /// Sync trackers to determin if new data has been produced by the encoders
    bool leftSync = false;
    bool rightSync = false;

    Position currentPosition = {0, 0, 0};
    Distance distance = {0, 0};

    std::map<Motor, float> currentReadings;
    std::map<Motor, float> lastReadings;
    std::map<Motor, float> totalDegreesTraveled;
    std::map<Motor, float> metersTraveledInFrame;
    std::map<Motor, float> totalMetersTraveled;

    /// Number of readings to throw out before considering the system stabilized
    int stablizationAmount = 3;

    // TODO change to left and right forward direction multiplier 1 or -1 contructor takes in
    bool rightIncrease;
    bool leftIncrease;

    // std::chrono::time_point<std::chrono::system_clock> linearStartTime;
    // std::chrono::time_point<std::chrono::system_clock> angularStartTime;

    // float maxVelocity = 0.0;
    // float linearVelocity = 0.0;
    // float angularVelocity = 0.0;

    // float totalDistance = 0.0;
};