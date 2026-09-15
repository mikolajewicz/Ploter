#pragma once

#include "MotorDriver.hpp"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"
#include "Homing.hpp"
#include "Kinematics.hpp"

#include <TMCStepper.h>

class MotionManager
{
private:
    MotorDriver& motor1;
    TMC2209Stepper& tmc1;
    MotionExecutor& motionExecutor1;
    TrajectoryGenerator& trajectoryGenerator1;
    Homing& homing1;

    MotorDriver& motor2;
    TMC2209Stepper& tmc2;
    MotionExecutor& motionExecutor2;
    TrajectoryGenerator& trajectoryGenerator2;
    Homing& homing2;

    Kinematics& solver;

public:
    MotionManager(
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

        Kinematics& solver
    );

    bool A2B(
        double pointA_x,
        double pointA_y,
        double pointB_x,
        double pointB_y,
        double time,
        double timeStep = 0.01
    );

    bool line(
        double pointA_x,
        double pointA_y,
        double pointB_x,
        double pointB_y,
        double speed, // mm/s
        double timeStep = 0.01);
};