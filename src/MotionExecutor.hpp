#pragma once

#include <vector>
#include <Arduino.h>

#include "MotorDriver.hpp"

class MotionExecutor
{
public:
    explicit MotionExecutor(MotorDriver& motorDriver)
        : motorDriver(motorDriver), intervalDurationUs(0) {}

    void start(const std::vector<int>& stepTrajectory, double timeStep);

    void update();

    bool hasActiveTrajectory() const {
        return active && trajectory != nullptr && !trajectoryBuffer.empty();
    }

private:
    const std::vector<int>* trajectory = nullptr;
    std::vector<int> trajectoryBuffer;

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
    
};