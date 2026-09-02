#include "MotionExecutor.hpp"
#include <Arduino.h>
#include <cstdlib>   // std::abs

void MotionExecutor::start(const std::vector<int>& stepTrajectory, double timeStep)
{
    trajectory = &stepTrajectory;

    currentInterval = 0;
    intervalDurationUs = static_cast<uint32_t>(timeStep * 1000000.0);

    intervalStart = micros();
    nextIntervalTime = intervalStart;

    stepsRemaining = 0;
    active = true;

     stepPin = motorDriver.getStepPin();
     dirPin = motorDriver.getDirPin();
}

void MotionExecutor::update()
{
    // Jeśli nie ma aktywnej trajektorii, to nic nie robimy.
    if (!active || trajectory == nullptr) {
        return;
    }

    // Która godzina?
    uint32_t currentTime = micros();

    if (currentInterval >= trajectory->size()) {
        active = false;
        return;
    }

    int stepValue = (*trajectory)[currentInterval];

    if (currentTime >= nextIntervalTime) {
        nextIntervalTime += intervalDurationUs;
        stepsRemaining = std::abs(stepValue);

        if (stepValue > 0) {
            digitalWrite(motorDriver.getDirPin(), HIGH);
        } else if (stepValue < 0) {
            digitalWrite(motorDriver.getDirPin(), LOW);
        }

        if (stepValue > 0) {
            motorDriver.step();
            stepPeriodUs = intervalDurationUs / stepValue;
        }

        nextStepTime = currentTime + stepPeriodUs;

        nextIntervalTime = currentTime + intervalDurationUs;
    }


     if (stepsRemaining != 0 && currentTime >= nextStepTime) {
        
        nextStepTime += stepPeriodUs;
        stepsRemaining--;

        motorDriver.step();
    }
}