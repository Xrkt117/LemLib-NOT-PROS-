#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "lemlib/platform.hpp"

void lemlib::Chassis::swingToPoint(float x, float y, DriveSide lockedSide, int timeout, SwingToPointParams params,
                                   bool async) {
    params.minSpeed = fabs(params.minSpeed);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        motionTask.start([this, x, y, lockedSide, timeout, params]() {
            swingToPoint(x, y, lockedSide, timeout, params, false);
        });
        this->endMotion();
        platform::delay(10);
        return;
    }
    float targetTheta;
    float deltaX, deltaY, deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    float prevRawDeltaTheta = 0;
    float prevDeltaTheta = 0;
    bool hasPrevRawDeltaTheta = false;
    bool hasPrevDeltaTheta = false;
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();
    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        // update variables
        Pose pose = getPose();
        pose.theta = (params.forwards) ? fmod(pose.theta, 360) : fmod(pose.theta - 180, 360);

        // update completion vars
        distTraveled = fabs(angleError(pose.theta, startTheta, false));

        deltaX = x - pose.x;
        deltaY = y - pose.y;
        targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (!hasPrevRawDeltaTheta) {
            prevRawDeltaTheta = rawDeltaTheta;
            hasPrevRawDeltaTheta = true;
        }
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (!hasPrevDeltaTheta) {
            prevDeltaTheta = deltaTheta;
            hasPrevDeltaTheta = true;
        }

        // motion chaining
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        infoSink()->debug("Turn Motor Power: {} ", motorPower);

        // move the drivetrain
        if (lockedSide == DriveSide::LEFT) {
            platform::drive(drivetrain.rightMotors, -motorPower);
            drivetrain.leftMotors->stop(vex::brakeType::hold);
        } else {
            platform::drive(drivetrain.leftMotors, motorPower);
            drivetrain.rightMotors->stop(vex::brakeType::hold);
        }

        platform::delay(10);
    }

    // stop the drivetrain
    platform::drive(drivetrain.leftMotors, 0);
    platform::drive(drivetrain.rightMotors, 0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}
