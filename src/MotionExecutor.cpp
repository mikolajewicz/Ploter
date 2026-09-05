#include "MotionExecutor.hpp"
#include <Arduino.h>
#include <cstdlib>   // std::abs
#include "TrajectoryGenerator.hpp"

void MotionExecutor::start(
    std::vector<int> stepTrajectory,
    double timeStep
)
{
    motorDriver.setMotionMode(MotorDriver::MotionMode::Trajectory);
    motorDriver.stop();

    // Take ownership of the provided trajectory by moving it into the buffer.
    trajectoryBuffer = std::move(stepTrajectory);
    trajectory = &trajectoryBuffer;

    currentInterval = 0;
    intervalDurationUs =
        static_cast<uint32_t>(timeStep * 1000000.0);

    nextIntervalTime = micros();
    stepsRemaining = 0;
    active = true;
}

void MotionExecutor::startFromGenerator(TrajectoryGenerator* generator, double timeStep, int stepsPerRevolution)
{
    motorDriver.setMotionMode(MotorDriver::MotionMode::Trajectory);
    motorDriver.stop();

    // Use position-based trajectory streamed from generator
    positionTrajectory = generator;
    usingPositionTrajectory = true;

    positionStepsPerRevolution = stepsPerRevolution;

    currentInterval = 0;
    intervalDurationUs = static_cast<uint32_t>(timeStep * 1000000.0);

    nextIntervalTime = micros();
    stepsRemaining = 0;
    active = true;

    // Initialize position state by fetching the first position if possible.
    haveLastPosition = false;
    residuePos = 0.0;
}

void MotionExecutor::update()
{
    // Jeśli nie ma aktywnej trajektorii, to nic nie robimy.
    if (!active) {
        return;
    }

    // Która godzina?
    uint32_t currentTime = micros();

    // --- Case A: step trajectory precomputed ---
    if (!usingPositionTrajectory) {
        if (trajectory == nullptr || trajectoryBuffer.empty()) {
            active = false;
            return;
        }

        if (static_cast<int32_t>(currentTime - nextIntervalTime) >= 0) {

            if (currentInterval >= trajectory->size()) {
                active = false;
                trajectory = nullptr;
                trajectoryBuffer.clear();
                return;
            }

            int stepValue = (*trajectory)[currentInterval];
            nextIntervalTime += intervalDurationUs;
            stepsRemaining = std::abs(stepValue);

            if (stepValue > 0) {
                motorDriver.setDirection(true);
            } else if (stepValue < 0) {
                motorDriver.setDirection(false);
            }

            if (stepsRemaining > 0) {
                stepPeriodUs =
                    intervalDurationUs /
                    static_cast<uint32_t>(stepsRemaining);

                motorDriver.step();
                stepsRemaining--;

                nextStepTime = currentTime + stepPeriodUs;
            } else {
                stepPeriodUs = 0;
            }
            currentInterval++;
        }

        if (stepsRemaining > 0 && static_cast<int32_t>(currentTime - nextStepTime) >= 0) {
            nextStepTime += stepPeriodUs;
            stepsRemaining--;

            motorDriver.step();
        }

        if (trajectory != nullptr && currentInterval >= trajectory->size() && stepsRemaining == 0) {
            active = false;
            trajectory = nullptr;
            trajectoryBuffer.clear();
            motorDriver.setMotionMode(MotorDriver::MotionMode::ConstantSpeed);
        }

        return;
    }

    // --- Case B: streaming position trajectory from generator ---
    if (usingPositionTrajectory) {
        if (positionTrajectory == nullptr) {
            active = false;
            return;
        }

        if (static_cast<int32_t>(currentTime - nextIntervalTime) >= 0) {
            // Need two successive positions to compute difference for this interval.
            double nextPos = 0.0;

            if (!haveLastPosition) {
                if (!positionTrajectory->getNextPosition(lastPosition)) {
                    // nothing to do
                    active = false;
                    positionTrajectory = nullptr;
                    usingPositionTrajectory = false;
                    motorDriver.setMotionMode(MotorDriver::MotionMode::ConstantSpeed);
                    return;
                }
                haveLastPosition = true;
            }

            if (!positionTrajectory->getNextPosition(nextPos)) {
                // no more points
                active = false;
                positionTrajectory = nullptr;
                usingPositionTrajectory = false;
                motorDriver.setMotionMode(MotorDriver::MotionMode::ConstantSpeed);
                return;
            }

            // compute difference between subsequent positions
            double diffAngle = nextPos - lastPosition;
            lastPosition = nextPos;

            nextIntervalTime += intervalDurationUs;

            double diffrence = diffAngle / 360.0 * static_cast<double>(positionStepsPerRevolution) + residuePos;
            int stepValue = static_cast<int>(diffrence);
            residuePos = diffrence - stepValue;

            stepsRemaining = std::abs(stepValue);

            if (stepValue > 0) {
                motorDriver.setDirection(true);
            } else if (stepValue < 0) {
                motorDriver.setDirection(false);
            }

            if (stepsRemaining > 0) {
                stepPeriodUs = intervalDurationUs / static_cast<uint32_t>(stepsRemaining);

                motorDriver.step();
                stepsRemaining--;

                nextStepTime = currentTime + stepPeriodUs;
            } else {
                stepPeriodUs = 0;
            }

            currentInterval++;
        }

        if (stepsRemaining > 0 && static_cast<int32_t>(currentTime - nextStepTime) >= 0) {
            nextStepTime += stepPeriodUs;
            stepsRemaining--;

            motorDriver.step();
        }

        return;
    }
}