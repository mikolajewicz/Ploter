#pragma once

#include <vector>
#include <Arduino.h>

#include "MotorDriver.hpp"

class TrajectoryGenerator;

class MotionExecutor
{
public:
    explicit MotionExecutor(MotorDriver& motorDriver)
        : motorDriver(motorDriver), intervalDurationUs(0) {}

    void start(std::vector<int> stepTrajectory, double timeStep);

    void update();

    bool hasActiveTrajectory() const {
        // Active either when using a precomputed step trajectory
        // or when streaming a position trajectory from a generator.
        if (!active) return false;
        if (usingPositionTrajectory) {
            return positionTrajectory != nullptr;
        }
        return trajectory != nullptr && !trajectoryBuffer.empty();
    }

private:
    const std::vector<int>* trajectory = nullptr;
    std::vector<int> trajectoryBuffer;
    
    // On-the-fly trajectory (positions)
    TrajectoryGenerator* positionTrajectory = nullptr;
    bool usingPositionTrajectory = false;
    double lastPosition = 0.0;
    bool haveLastPosition = false;
    double residuePos = 0.0;
    int positionStepsPerRevolution = 1600;

    MotorDriver& motorDriver;

    size_t currentInterval = 0;

    uint32_t intervalStart = 0;
    uint32_t nextIntervalTime = 0;
    uint32_t nextStepTime = 0;

    uint32_t intervalDurationUs = 0;
    uint32_t stepPeriodUs = 0;

    int stepsRemaining = 0;

    bool direction = true;
    bool active = false;
    
public:
    void startFromGenerator(const TrajectoryGenerator* generator, double timeStep);
    void startFromGenerator(TrajectoryGenerator* generator, double timeStep, int stepsPerRevolution);
};