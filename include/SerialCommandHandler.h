#pragma once

#include <Arduino.h>
#include <TMCStepper.h>

#include "MotorDriver.hpp"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"
#include "Homing.hpp"

class MotionManager;

class SerialCommandHandler
{
private:
    MotorDriver* motors_[2];
    TMC2209Stepper* tmcs_[2];
    MotionExecutor* motionExecutors_[2];
    TrajectoryGenerator* trajectoryGenerators_[2];
    Homing* homings_[2];

    MotionManager& motionManager;

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

    void handleSerialCommand(
        String line
    );

    void handleMotorCommand(
        uint8_t motorIndex,
        String line
    );

    void handleTabletCommand(
        const String& line
    );

     void handleMotionCommand(
        String line
    );

    void updatePendingMotion();

    bool motionCommandPending_ = false;
    uint32_t motionCommandQueuedAt_ = 0;

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

    void printStatus(
        uint8_t motorNumber
    );

    void stopAll();

    // Na razie zostawiamy stare funkcje.
    // Posprzatamy je w kolejnym kroku.

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
};