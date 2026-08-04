#pragma once

#include <Arduino.h>

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

private:
    int stepPin;
    int dirPin;
    int enablePin;

    uint32_t speedStepsPerSecond;
    uint32_t halfPeriodMicroseconds;
    uint32_t previousToggleTime;

    bool direction;
    bool enabled;
    bool running;
    bool stepState;
};