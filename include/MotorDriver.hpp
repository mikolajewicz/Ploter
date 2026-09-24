#pragma once

#include <Arduino.h>
#include <vector>

class MotorDriver {
public:
    MotorDriver(
        int stepPin,
        int dirPin,
        int enablePin = -1
    );

    void begin();

    void enable();
    void disable();

    void start();
    void stop();

    void setDirection(bool direction);
    void setSpeed(uint32_t stepsPerSecond);

    void run();

    bool isEnabled() const;
    bool isRunning() const;

    uint32_t getSpeed() const;
    bool getDirection() const;

    uint8_t getStepPin() const { return stepPin; }
    uint8_t getDirPin() const { return dirPin; }

    void step();

    int64_t getStepCount(){ return stepCount; }
    void setStepCount(int newStepCount){ stepCount = newStepCount;}
    void resetStepCount()
    {
        stepCount = 0;
    }

private:
    int stepPin;
    int dirPin;
    int enablePin;

    int64_t stepCount = 0;

    uint32_t speedStepsPerSecond;
    uint32_t halfPeriodMicroseconds;
    uint32_t previousToggleTime;

    bool direction;
    bool enabled;
    bool running;
    bool stepState;
};