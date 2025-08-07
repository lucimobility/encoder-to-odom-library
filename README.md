# Encoder-to-Odometry Library

C++ library to convert differential drive encoder readings into accurate position, velocity, and distance measurements.

Incoming data should be the two wheel encoder angle readings and the time stamp of the data.

Output data will be standard odometry data
(X,Y,Z, Theta position data) (X,Y,Z velocity data)

## Hardware

- AS5600 magnetic encoders (12-bit resolution: 0-4096 values)
- Arduino/STM32 microcontrollers with `millis()` timestamps

## Coordinate System

This library follows the standard robotics right-hand rule:

- **X-axis**: Forward/backward (positive = forward direction)
- **Y-axis**: Left/right (positive = left side)
- **Z-axis**: Up/down (rotation about Z = yaw)
- **Theta**: Counter-clockwise rotation (positive = turning left)

```
                  X (forward)
                  ↑
    (counter-     │
     clockwise) θ │
                ↖ │
   Y   ←──────────┼───
 (left)           |
```

## Building

This library is built with cmake, to build run the following:

```
git clone https://github.com/lucimobility/encoder-to-odom-library.git
cd encoder-to-odom-library
mkdir build && cd build
cmake ..
make -jn (n is number of parallel jobs you want a good value is usually 4)
```

## Including Library in a Project

### CMake FetchContent

This library can be added to any existing project by either cloning into that project, making it a submodule, or the recommended method if your project uses CMake which is FetchContent command in a CMake file:

```
include(FetchContent)
set(NO_TESTS TRUE CACHE INTERNAL "Disable tests in fetched content")
# Clone the odom library
FetchContent_Declare(encoder_to_odom
    GIT_REPOSITORY git@github.com:lucimobility/encoder-to-odom.git
    GIT_TAG <latest git commit hash>
)
# Make the library available to the system
FetchContent_MakeAvailable(encoder_to_odom)
```

### Conan

If using conan, this library can also be made into a conan package:

```
# Create conan package from project root
conan create . --build=missing

# upload package to your conan artifactory
conan upload encoder-to-odom/<version> --remote=<remote conan artifactory> --confirm
```

## Testing

This library uses gtesting suit to run unit tests. To run tests follow the below instructions.

Run

```
# After building
ctest

# Expected output:
#     Start 1: FrameTests.UpdateReadings
# 1/8 Test #1: FrameTests.UpdateReadings ..................   Passed    0.00 sec
# ...
# 100% tests passed, 0 tests failed out of 8
# Total Test time (real) =   0.03 sec
```

## Quick Start

```cpp
#include "odometry.h"

// Initialize with your robot parameters
OdometryProcessor odom(
    0.314,    // wheel circumference (meters) - π × 0.1m diameter
    0.5,      // wheelbase (meters) - distance between wheel centers
    2.38,     // gear ratio - encoder rotations per wheel rotation
    100.0,    // rollover threshold (degrees)
    true,     // right motor forward increases encoder values
    false     // left motor forward decreases encoder values
);

// Main control loop
while (running) {
    // Update encoder readings (in degrees)
    odom.updateEncoderReading(Motor::LEFT, leftEncoderAngle);
    odom.updateEncoderReading(Motor::RIGHT, rightEncoderAngle);
    odom.updateTimestamp(millis()); // Arduino timestamp in milliseconds

    // Process the data
    odom.processData();

    // Get results
    Position pos = odom.getPosition();
    Velocity vel = odom.getVelocity();
    Distance dist = odom.getDistance();
}
```

## Configuration Parameters

```cpp
OdometryProcessor(
    float wheelCircumference,     // meters
    float wheelBase,             // meters
    float gearRatio,             // dimensionless
    float rolloverThreshold,     // degrees
    bool rightMotorForwardIncreases = true,
    bool leftMotorForwardIncreases = true
);
```

| Parameter                      | Type    | Description                                                     |
| ------------------------------ | ------- | --------------------------------------------------------------- |
| **wheelCircumference**         | `float` | Circumference of drive wheels in meters                         |
| **wheelBase**                  | `float` | Distance between wheel centers in meters                        |
| **gearRatio**                  | `float` | Encoder rotations per wheel rotation                            |
| **rolloverThreshold**          | `float` | Max angle change per frame before triggering rollover detection |
| **rightMotorForwardIncreases** | `bool`  | True if right encoder increases when moving forward             |
| **leftMotorForwardIncreases**  | `bool`  | True if left encoder increases when moving forward              |

## Documentation Generation

All code here is documented with Doxygen. In order to build the docs you must first have Doxygen and graphviz installed. It is available through the debian/ubuntu repositories.

```bash
# Generate documentation
cd docs/
doxygen
```

This should generate both HTML and LaTeX output. To view the HTML, simply navigate to `file:///path/to/encoder-to-odom-library/docs/html/index.html` in your web browser of choice.

## Mathematical Documentation

The odometry calculation process consists of six main steps:

1. **Encoder Rollover Detection** - Handle 360°/0° boundary crossings
2. **Distance Conversion** - Convert encoder angles to wheel distances
3. **Robot Distance Calculation** - Compute center point movement
4. **Heading Calculation** - Determine orientation change using differential kinematics
5. **Position Update** - Use dual-algorithm approach for optimal accuracy
6. **Velocity Calculation** - Compute linear and angular velocities

### Step 1: Encoder Rollover Detection

**The Problem:** Encoders wrap around at 360°/0°, causing apparent large movements:

- Actual movement: 350° → 10° (forward 20°)
- Raw calculation: 10° - 350° = -340° (backward 340°) ❌

**Detection Algorithm:**

```cpp
float angleDelta = currentAngle - previousAngle;

if (angleDelta > rolloverThreshold) {
    // Rollunder detected: 350° → 10°
    angleDelta = angleDelta - 360.0f; // -340° becomes +20° ✓
} else if (angleDelta < -rolloverThreshold) {
    // Rollover detected: 10° → 350°
    angleDelta = angleDelta + 360.0f; // +340° becomes -20° ✓
}
```

| Previous | Current | Raw Delta | Detected  | Corrected |
| -------- | ------- | --------- | --------- | --------- |
| 350°     | 10°     | -340°     | Rollunder | +20° ✓    |
| 10°      | 350°    | +340°     | Rollover  | -20° ✓    |
| 100°     | 120°    | +20°      | Normal    | +20° ✓    |

### Step 2: Distance Conversion Pipeline

**Encoder → Wheel Distance Conversion:**

```cpp
// 1. Convert encoder degrees to rotations
float encoderRotations = angleDelta / 360.0f;

// 2. Account for gear reduction
float wheelRotations = encoderRotations / gearRatio;

// 3. Handle motor direction (some count backwards when moving forward)
if (!motorForwardIncreases) {
    wheelRotations = -wheelRotations;
}

// 4. Convert rotations to linear distance
float distanceMeters = wheelRotations * wheelCircumference;
```

### Step 3: Robot Center Distance

```cpp
// Robot center moves at average of both wheels
float robotDistance = (leftWheelDistance + rightWheelDistance) / 2.0f;
```

### Step 4: Heading Calculation (Differential Drive Kinematics)

**Basic Differential Drive Formula:**

```cpp
float deltaTheta = (rightDistance - leftDistance) / wheelBase;
```

### Step 5: Advanced Position Calculation - Dual Algorithm Approach

The library automatically selects between two algorithms based on turning magnitude:

#### Small Angle Approximation (< 0.57°)

For small turns, uses linear approximation with midpoint orientation:

```cpp
float midTheta = currentPosition.theta - deltaTheta / 2.0f;
deltaX = robotDistance * cosf(midTheta);
deltaY = robotDistance * sinf(midTheta);
```

#### Large Angle Exact Geometry (≥ 0.57°)

For larger turns, uses exact arc geometry to prevent drift:

```cpp
float radius = robotDistance / deltaTheta; // Instantaneous turning radius
float prevTheta = currentPosition.theta - deltaTheta;

// Exact arc geometry integration
deltaX = radius * (sinf(currentPosition.theta) - sinf(prevTheta));
deltaY = radius * (cosf(prevTheta) - cosf(currentPosition.theta));
```

**Visual Representation:**

```
Large Turn Path (Exact Arc Geometry):
      ╭─────────╮
     ╱           ╲     ← Prevents position drift
    ╱             ╲      during curved motion
   ╱               ╲
  ╱       ICR       ╲   ICR = Instantaneous Center
 ╱         •         ╲      of Rotation
╱                     ╲
Start ─────────────── End
      Robot Path
```

**Why 0.57°?** Based on error analysis, below this threshold the linear approximation error is negligible compared to typical encoder noise.

### Step 6: Velocity Calculations

**Linear Velocity:**

```cpp
float deltaTimeSeconds = deltaTime / 1000.0f;
velocity.linearX = robotDistance / deltaTimeSeconds; // m/s
```

**Angular Velocity:**

```cpp
velocity.angularZ = deltaTheta / (deltaTime / 1000.0f); // rad/s
```

**Noise Characteristics:** Velocity naturally has more noise than position since it's the derivative of sensor measurements. Consider adding filtering for smoother velocity output.

## Troubleshooting

### Velocity Issues

**Problem: NaN or Inf velocity values**

- **Cause:** Missing or inconsistent timestamps
- **Solution:** Ensure `updateTimestamp()` is called before each `processData()`
- **Check:** Verify timestamp increases monotonically

**Problem: Noisy velocity readings**

- **Cause:** Low encoder update rate or timing jitter
- **Solution:** Increase update frequency to >50Hz, add velocity filtering
- **Check:** Monitor `getDeltaTime()` for consistency

### Position Issues

**Problem: Robot drifts during straight-line motion**

- **Cause:** Incorrect wheel circumference or motor direction settings
- **Solution:** Verify wheel parameters, verify `motorForwardIncreases` flags
- **Test:** Drive robot in perfect straight line, check Y position remains ~0

**Problem: Incorrect turning radius**

- **Cause:** Wrong wheelbase measurement
- **Solution:** Measure distance between wheel centers, not wheel edges
- **Test:** Command 360° turn, verify robot returns to start position
