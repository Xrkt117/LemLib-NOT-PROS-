#include "lemlib/chassis/trackingWheel.hpp"

#include <math.h>

lemlib::TrackingWheel::TrackingWheel(vex::encoder* encoder, float wheelDiameter, float distance, float gearRatio)
    : diameter(wheelDiameter),
      distance(distance),
      rpm(0),
      cartridgeRpm(0),
      encoder(encoder),
      gearRatio(gearRatio) {}

lemlib::TrackingWheel::TrackingWheel(vex::rotation* encoder, float wheelDiameter, float distance, float gearRatio)
    : diameter(wheelDiameter),
      distance(distance),
      rpm(0),
      cartridgeRpm(0),
      rotation(encoder),
      gearRatio(gearRatio) {}

lemlib::TrackingWheel::TrackingWheel(vex::motor_group* motors, float wheelDiameter, float distance, float rpm,
                                     float cartridgeRpm)
    : diameter(wheelDiameter),
      distance(distance),
      rpm(rpm),
      cartridgeRpm(cartridgeRpm),
      motors(motors) {}

void lemlib::TrackingWheel::reset() {
    if (encoder != nullptr) encoder->resetRotation();
    if (rotation != nullptr) rotation->resetPosition();
    if (motors != nullptr) motors->resetPosition();
}

float lemlib::TrackingWheel::getDistanceTraveled() {
    if (encoder != nullptr) {
        return (encoder->position(vex::degrees) * diameter * M_PI / 360.0) / gearRatio;
    }
    if (rotation != nullptr) {
        return (rotation->position(vex::degrees) * diameter * M_PI / 360.0) / gearRatio;
    }
    if (motors != nullptr) {
        const float gearing = cartridgeRpm == 0 ? 1 : rpm / cartridgeRpm;
        return motors->position(vex::degrees) * diameter * M_PI / 360.0 * gearing;
    }
    return 0;
}

float lemlib::TrackingWheel::getOffset() { return distance; }

int lemlib::TrackingWheel::getType() { return motors != nullptr ? 1 : 0; }
