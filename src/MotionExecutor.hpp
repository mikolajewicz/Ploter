#pragma once

#include <vector>
#include <Arduino.h>

#include "MotorDriver.hpp"

class MotionExecutor
{
public:
    explicit MotionExecutor(MotorDriver& motorDriver, double timeStep = 0.0)
        : motorDriver(motorDriver), intervalDurationUs(timeStep > 0.0 ? static_cast<uint32_t>(timeStep * 1000000.0) : 0) {}

    void start(const std::vector<int>& stepTrajectory, double timeStep);

    void update();

    void setTimeStep(double timeStep) {
        intervalDurationUs = static_cast<uint32_t>(timeStep * 1000000.0);
    }

private:
    const std::vector<int>* trajectory = nullptr;

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