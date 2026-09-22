#pragma once

#include <Arduino.h>
#include <TMCStepper.h>

#include "MotorDriver.hpp"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"
#include "Homing.hpp"
#include "Kinematics.hpp"

// Forward declaration to avoid including MotionManager header here
class MotionManager;

class SerialCommandHandler {
private:
    MotorDriver* motors_[2];
    TMC2209Stepper* tmcs_[2];
    MotionExecutor* motionExecutors_[2];
    TrajectoryGenerator* trajectoryGenerators_[2];
    Homing* homings_[2];

    uint8_t selectedMotor_ = 0;

    String serialCommand_;

    bool stopped_ = false;

    bool parseLongArgument(
        const String& argument,
        long& result
    );

    bool parseDoubleArgument(
        const String& argument,
        double& result
    );

    void handleSerialCommand(String line);

    MotionManager& motionManager;
public:
    SerialCommandHandler(
        MotorDriver& motor1,
        TMC2209Stepper& tmc1,
        MotionExecutor& motionExecutor1,
        TrajectoryGenerator& trajectoryGenerator1,
        Homing& homing1,

        MotorDriver& motor2,
        TMC2209Stepper& tmc2,
        MotionExecutor& motionExecutor2,
        TrajectoryGenerator& trajectoryGenerator2,
        Homing& homing2,

        MotionManager& motionManager
    );

    void readSerialCommands();

    void printHelp();
    void printStatus();

    bool trapeze(
        uint8_t motor,
        double distance,
        double time,
        double accelerationTime,
        double timeStep
    );

    bool cosine(
        uint8_t motor,
        double amplitude,
        double frequency,
        double duration,
        double timeStep
    );

    void stopAll();

    bool A2B(
        double pointA_x,
        double pointA_y,
        double pointB_x,
        double pointB_y,
        double time,
        double timeStep
    );

    bool line(
        double ax,
        double ay,
        double bx,
        double by,
        double speed,
        double timeStep
    );

    void handleTabletCommand(const String& line);

};