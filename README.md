# encoder-to-odom-library
C++ library to handle converting encoder angle readings to odom values


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

## Documentation generation
TODO

## Documentation of the math
TODO