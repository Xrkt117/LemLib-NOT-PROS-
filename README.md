# LemLib for VEXcode C++

This fork runs on a VEX V5 Brain with the official VEXcode C++ SDK. It does not use PROS.

## Open and build

1. Install the **VEX Robotics** extension in VS Code.
2. Open this repository as a folder.
3. Connect the V5 Brain by USB.
4. Use the VEX extension's **Build** or **Build and Download** command.

The included `src/main.cpp` is a complete six-motor drivetrain example. Change its ports, reversed flags, wheel size,
track width, gear cartridge speed, and PID values for your robot before downloading it.

## No namespace punctuation in robot code

Include `lemlib.h`, then use ordinary VEX and LemLib names directly:

```cpp
#include "lemlib.h"

motor leftMotor(PORT1, ratio6_1, false);
motor rightMotor(PORT2, ratio6_1, true);
motor_group leftMotors(leftMotor);
motor_group rightMotors(rightMotor);
inertial inertialSensor(PORT10);

Drivetrain driveConfig(&leftMotors, &rightMotors, 12, 4, 600, 2);
OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &inertialSensor);
```

No namespace qualifier is needed in your robot program.

Named turn and swing choices are also available without punctuation, for example
`chassis.swingToHeading(90, leftSide, 1500)` and `params.direction = clockwise`.

Pure-pursuit paths can be embedded as ordinary C++ text:

```cpp
PATH_ASSET(testPath, "0, 0, 60\n0, 24, 60\nendData\n");

void autonomous() {
    chassis.follow(testPath, 10, 3000);
}
```

## Important drivetrain value

The `rpm` argument is the actual wheel speed after external gearing. The optional final `cartridgeRpm` argument defaults
to 600. Supply it when you use a 100 RPM or 200 RPM cartridge so motor-integrated odometry scales correctly.

This is an independent compatibility port and is not an official LemLib release.
