#include "lemlib.h"

brain Brain;
controller Controller;
competition Competition;

motor leftFront(PORT1, ratio6_1, true);
motor leftMiddle(PORT2, ratio6_1, false);
motor leftBack(PORT3, ratio6_1, true);
motor rightFront(PORT4, ratio6_1, false);
motor rightMiddle(PORT5, ratio6_1, true);
motor rightBack(PORT6, ratio6_1, false);

motor_group leftMotors(leftFront, leftMiddle, leftBack);
motor_group rightMotors(rightFront, rightMiddle, rightBack);
inertial inertialSensor(PORT10);

Drivetrain driveConfig(&leftMotors, &rightMotors, 12.0, 4.0, 600, 2);

ControllerSettings lateralController(10, 0, 3, 3, 1, 100, 3, 500, 5);
ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);

OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &inertialSensor);
ExpoDriveCurve throttleCurve(3, 10, 1.019);
ExpoDriveCurve steerCurve(3, 10, 1.019);

Chassis chassis(driveConfig, lateralController, angularController, sensors, &throttleCurve, &steerCurve);

void updateBrainScreen() {
    while (true) {
        Pose pose = chassis.getPose();
        const char* mode = "Mode: Disabled";
        if (Competition.isAutonomous()) mode = "Mode: Autonomous";
        else if (Competition.isDriverControl()) mode = "Mode: Driver";

        Brain.Screen.clearScreen();
        Brain.Screen.setFont(mono20);
        Brain.Screen.printAt(20, 30, "LemLib VEXcode");
        Brain.Screen.printAt(20, 70, "X: %.2f in", pose.x);
        Brain.Screen.printAt(20, 105, "Y: %.2f in", pose.y);
        Brain.Screen.printAt(20, 140, "Heading: %.1f deg", pose.theta);
        Brain.Screen.printAt(20, 185, mode);
        Brain.Screen.render();

        wait(100, msec);
    }
}

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000);
    chassis.turnToHeading(90, 1500);
}

void driverControl() {
    while (true) {
        int throttle = Controller.Axis3.position();
        int turn = Controller.Axis1.position();
        chassis.arcade(throttle, turn);
        wait(10, msec);
    }
}

int main() {
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(mono20);
    Brain.Screen.printAt(20, 40, "Calibrating inertial...");
    Brain.Screen.render();

    chassis.calibrate();
    thread screenThread(updateBrainScreen);
    Competition.autonomous(autonomous);
    Competition.drivercontrol(driverControl);

    while (true) wait(100, msec);
}
