#pragma once

#include <vector>
#include <Arduino.h>

#include "MotorDriver.hpp"

class MotionExecutor
{
public:
    MotionExecutor(MotorDriver& motorDriver)
        : motorDriver(motorDriver) {}
    void start(const std::vector<int>& stepTrajectory, double timeStep);
    void update();

private:
    const std::vector<int>* trajectory = nullptr;

    MotorDriver& motorDriver;
    uint8_t stepPin;
    uint8_t dirPin;

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