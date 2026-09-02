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

}

void MotionExecutor::update()
{
    // Jeśli nie ma aktywnej trajektorii, to nic nie robimy.
    if (!active || trajectory == nullptr) {
        return;
    }

    // Która godzina?
    uint32_t currentTime = micros();

    if (static_cast<int32_t>(currentTime - nextIntervalTime) >= 0) {

        if (currentInterval >= trajectory->size()) {
        active = false;
        return;
        }

        int stepValue = (*trajectory)[currentInterval];
        nextIntervalTime += intervalDurationUs;
        stepsRemaining = std::abs(stepValue);

        if (stepValue > 0) {
            motorDriver.setDirection(true);
        } else if (stepValue < 0) {
            motorDriver.setDirection(false);
        } else {
            // Jeśli stepValue == 0, to nie zmieniamy kierunku.
        }

        if (stepsRemaining > 0) {
            stepPeriodUs =
            intervalDurationUs /
            static_cast<uint32_t>(stepsRemaining);

            motorDriver.step();
            stepsRemaining--;

            nextStepTime = currentTime + stepPeriodUs;
        } 
        else {
            stepPeriodUs = 0;
        }
        currentInterval++;
    }

    if (stepsRemaining > 0 &&static_cast<int32_t>(currentTime - nextStepTime) >= 0) {
        
        nextStepTime += stepPeriodUs;
        stepsRemaining--;

        motorDriver.step();
    }


}