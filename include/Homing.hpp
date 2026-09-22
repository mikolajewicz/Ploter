#pragma once

#include <Arduino.h>
#include <TMCStepper.h>

#include "MotorDriver.hpp"

class Homing {
private:
    size_t homingState = 0;

    uint8_t stallThreshold = 50;

    int diagPin;
    bool direction;
    bool active = false;
    uint32_t homingStart = 0;

    MotorDriver& motor;
    TMC2209Stepper& tmc;

public:
    Homing(
        MotorDriver& motor,
        TMC2209Stepper& tmc,
        bool direction,
        int diagPin
    )
        : motor(motor),
          tmc(tmc),
          diagPin(diagPin),
          direction(direction)
    {
    }

    bool update();

    void home(){
        homingState = 1;
        active = true;
    }

    bool isActive() const {
        return active;
    }
};