# encoder-to-odom-library

C++ library to handle converting encoder angle readings to odom values. This library expects that your encoders are producing angle readings corresponding with individual specific angles. That is to say when tested with an AS5600 encoder chip a the wheel degrees gets read as 0-4096 unique values that the encoder will send back. It is also assumed that the edge board reading the encoder and feeding this library provides a constant clock reading (in ms) that corresponds with the time the sensor was taken if using an arduino one method is to use the millis() function. Please note that this clock is only needed if calculating velocity, NOT if just position is needed.

Incoming data should be the two wheel encoder angle readings and the time stamp of the data.

Output data will be standard odometry data
(X,Y,Z, Theta position data) (X,Y,Z velocity data)

The system also assumes a standard right hand rule axis setup on your robot where positive X is going forward, positive Y is left, and positive Z is turning left.

## Building

This library is built with cmake, to build run the following

```
git clone https://github.com/lucimobility/encoder-to-odom-library.git
cd encoder-to-odom-library
mkdir build
cd build
cmake ..
make -jn (n is number of parallel jobs you want a good value is usually 4)
```

## Including Library in a Project
This library can be added to any existing project by either cloning into that project, making it a submodule, or the recommended method if your project uses CMake which is FetchContent command in a CMake file. 

## Testing

This library uses gtesting suit to run unit tests. To run tests follow the below instructions.

Build the library using the instruction above.

Run

```
ctest
```

Output should show all tests passing

```
    Start 1: FrameTests.UpdateReadings
1/8 Test #1: FrameTests.UpdateReadings ..................   Passed    0.00 sec
    Start 2: FrameTests.Settled
2/8 Test #2: FrameTests.Settled .........................   Passed    0.00 sec
    Start 3: FrameTests.DeltaDegrees
3/8 Test #3: FrameTests.DeltaDegrees ....................   Passed    0.00 sec
    Start 4: FrameTests.RollOver
4/8 Test #4: FrameTests.RollOver ........................   Passed    0.00 sec
    Start 5: FrameTests.Velocity
5/8 Test #5: FrameTests.Velocity ........................   Passed    0.00 sec
    Start 6: TotalTests.DeltaMetersOneEncoderRotation
6/8 Test #6: TotalTests.DeltaMetersOneEncoderRotation ...   Passed    0.00 sec
    Start 7: TotalTests.AngleTest
7/8 Test #7: TotalTests.AngleTest .......................   Passed    0.00 sec
    Start 8: TotalTests.PositionTest
8/8 Test #8: TotalTests.PositionTest ....................   Passed    0.00 sec

100% tests passed, 0 tests failed out of 8

Total Test time (real) =   0.03 sec
```

## Example

Examples of how to setup and run the code can be found in the unit tests in the /test folder. The main principles is as follows.

1. Setup a
```
 OdometryProcessor(float wheelCircumference, float wheelBase, float gearRatio,
                  float rolloverThreshold, bool rightIncrease = true, bool leftIncrease = true);
```
object where the parameters are as follow.

| Parameter         | Type  | Description                                                                                                                        |
| ----------------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------- |
| wheelCircumference  | float | The circumference in meters of the drive wheels                                                                                    |
| wheelBase         | float | The distance in meters between the two drive wheels on their drive axis (width of robot)                                           |
| gearRatio       | float | number of encoder readings to wheel rotations Ex. 761 encoder readings for every 360 degree wheel readings this value would be 2.1 |
| rolloverThreshold | float | The highest delta reading that can be seen between frames that is considered NOT a rollover / rollunder                            |
| rightIncrease     | bool  | True if the right wheel rotating forward yields and increase in encoder readings                                                   |
| leftIncrease      | bool  | True if the left wheel rotating forward yields and increase in encoder readings                                                    |

2. When encoder readings come in call the `void updateCurrentValue(Motor motor, float value);` member function per wheel reading. 
3. Call `processData()` member function.
4. Use getters to read out the values you care about. 
   1. `getPosition()`
   2. `getVelocity()`

## Documentation Generation

All code here is documented with Doxygen. In order to build the docs you must first have Doxygen and graphviz installed. It is available through the debian/ubuntu repositories. After this, go to the docs/ directory, and run `doxygen`. This should generate both HTML and LaTeX output. To view the HTML, simply navigate to file:///path/to/encoder-to-odom-library/docs/html/index.html in your web browser of choice.

## Documentation of the Math

Wheel odometry can be a complex topic but if you break it down into its discrete steps it becomes very manageable. This library breaks the process into ~ 6 steps that we will discuss below.

1.  Calculate the distance traveled by each wheel in a single frame.
2.  Calculate the total distance the robot has traveled in a single frame.
3.  Calculate the change in heading in a single frame.
4.  Calculate the distance moved in the x, y direction.
5.  Update total pose estimate with frame values calculated.
6.  Calculate velocity based on delta time from sensor readings.

### Calculate Distance Traveled of Each Wheel

When both wheels have been updated with new encoder readings the system triggers a callback to process how far the wheels have traveled since last update. This is broken into subitems that involve first finding the encoder degrees traveled, then the wheel angle traveled, then the distance traveled from wheel angle.

The first step of finding the angle the encoder traveled uses a simple delta calculation but also takes into account rollover/ rollunder. This occurs when the encoder reading goes from max -> min or min -> max reading respectfully. This needs to be handled so the system doesn't think a wheel suddenly went many rotations in a single frame. This is handled by checking if the delta angle between frames is greater then a given value. This value is the rollover threshold and is tunable based on max velocity expected from a robot. Too high and you may miss rollover, too low and real values may trick the system into thinking it rolled over.

Once the system can determine the encoder rotations it uses the user provided encoder to wheel ratio value to find the angle rotated by the wheels. This can then be used with the wheel circumstance to find the number of meters traveled in a single frame.

### Calculate the Total Distance Traveled by System

Once the individual wheel distance is found the total systems distance traveled in a single frame can be found with the equation.

```
frameDistance = (rightDistance + leftDistance) / 2.0;
```

This is just an average of each wheel and for differential drive systems will work out to be the distance the point between the two wheels traveled.

### Calculate the Change in Heading in a Single Frame

To calculate the change in heading in a single frame you only need to know the distance traveled by each wheel and the wheel base distance, that is the distance between the two wheels.

With all three values the math is very simple.

```
    // Delta between two motors traveled
    float difference = rightDistance - leftDistance;

    float angle = asinf(difference / this->wheelBase); // Radians
```

This takes the difference between both wheels and some basic trig to find a frame delta theta.

### Calculate the Distance Moved in X,Y Directions

Now that we have the delta theta and delta x, y we can do some basic trig once more to find where the system is on an x,y plane.

For the X direction

```
float distanceMoved = cosf(this->currentPosition.theta) * this->distance.frameDistance;
```

For the Y direction

```
float distanceMoved = sinf(this->currentPosition.theta) * this->distance.frameDistance;
```

### Calculate the Total Pose

Once you have a single frame reading finding the pose since power on is simply found by adding up incremental frame values.
In practice the code does this as it calculates each component and is not a discrete step.

### Calculate the Velocity (Optional)

If providing the library with an updating system clock velocity is easily found by dividing the frame delta Y and Theta values by the delta time between the last timestamp.

Note: This has a natural level of noise it created as taking a derivative of a sensor value will cause.

## Troubleshooting

**My velocity is reporting as Nan or Inf:**

- This occurs when the library is not being fed a clock time to use when calculating the velocity. If you only want position data and not velocity these readings are fine.

**My distance traveled on a straight line is incorrect:**

- This can be any number of reasons but the main seen from experience is that encoders are not reporting values fast enough. This library has been tested to work well if new data makes it to the system at ~ 50Hz but faster is always better!
