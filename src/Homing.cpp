#include "Homing.hpp"
#include <TMCStepper.h>

bool Homing::update()
{
    if (!active) {
        return false;
    }

    switch (homingState) {

    case 1: {
        tmc.rms_current(500);
        tmc.en_spreadCycle(false);
        tmc.pwm_autoscale(true);

        tmc.TCOOLTHRS(0xFFFFF);
        tmc.SGTHRS(stallThreshold);

        motor.setDirection(direction);
        motor.setSpeed(1500);

        homingStart = millis();

        homingState = 2;
        break;
    }

    case 2: {
        // najpierw jedź przez 300 ms
        // i ignoruj DIAG
        if (millis() - homingStart < 300) {
            break;
        }

        if (digitalRead(diagPin) == HIGH) {
            motor.stop();

            active = false;
            homingState = 0;

            return true;
        }

        break;
    }

    default: {
        motor.stop();

        active = false;
        homingState = 0;

        break;
    }
    }

    return false;
}