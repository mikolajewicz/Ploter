#pragma once

#include <TMCStepper.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "MotorDriver.hpp"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"
#include "Homing.hpp"
#include "Kinematics.hpp"

#include <vector>



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

    std::vector<int> nextTrajectory1;
    std::vector<int> nextTrajectory2;

    double nextTimeStep = 0.01;

    bool trajectoryReady = false;

    enum class MotionType
    {
        A2B,
        LINE
    };

    struct MotionRequest{
        MotionType type;

        double pointA_x;
        double pointA_y;

        double pointB_x;
        double pointB_y;

        double value;    // A2B  -> time, LINE -> speed

        double timeStep;
    };

    QueueHandle_t motionQueue = nullptr;
    TaskHandle_t plannerTaskHandle = nullptr;
    SemaphoreHandle_t trajectoryMutex = nullptr;

    static void plannerTaskEntry(void* parameter);
    void plannerTask();

    double liveTargetX = 0.0;
    double liveTargetY = 0.0;

    double livePressure = 0.0;

    bool liveTargetInside = false;
    bool liveTargetReceived = false;

    uint32_t liveTargetLastUpdate = 0;

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

    bool startPreparedMotion();

    bool isTrajectoryReady();

    bool beginPlanner();

    bool planA2B(
        double pointA_x,
        double pointA_y,
        double pointB_x,
        double pointB_y,
        double time,
        double timeStep
    );

    bool planLine(
    double pointA_x,
    double pointA_y,
    double pointB_x,
    double pointB_y,
    double speed,
    double timeStep
    );

    void setLiveTarget(
        double x,
        double y,
        double pressure,
        bool inside
    );
};