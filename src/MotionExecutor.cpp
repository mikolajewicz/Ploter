#include "MotionExecutor.hpp"
#include <Arduino.h>
#include <cstdlib>   // std::abs

void MotionExecutor::start(
    std::vector<int>&& stepTrajectory,
    double timeStep
)
{
    trajectory = std::move(stepTrajectory);

    currentInterval = 0;
    intervalDurationUs = static_cast<uint32_t>(timeStep * 1000000.0);

    intervalStart = micros();
    nextIntervalTime = intervalStart;

    stepsRemaining = 0;
    active = true;

}


void MotionExecutor::update()
{
    if (!active)
    {
        return;
    }

    if (!motorDriver.isEnabled() || intervalDurationUs == 0)
    {
        stop();
        return;
    }

    uint32_t now = micros();

    // ----------------------------------------
    // 1. Wykonaj kroki bieżącego przedziału
    // ----------------------------------------

    if (stepsRemaining > 0)
    {
        if (
            static_cast<int32_t>(
                now - nextStepTime
            ) >= 0
        )
        {
            motorDriver.step();
            stepsRemaining--;

            // Termin następnego kroku
            nextStepTime += stepPeriodUs;

            // Jeśli jesteśmy już spóźnieni,
            // nie nadrabiaj kroków serią impulsów.
            uint32_t afterStep = micros();

            if (
                static_cast<int32_t>(
                    afterStep - nextStepTime
                ) >= 0
            )
            {
                nextStepTime =
                    afterStep + stepPeriodUs;
            }
        }

        // Nie przechodzimy do następnego przedziału,
        // dopóki nie wykonamy wszystkich kroków.
        return;
    }

    // ----------------------------------------
    // 2. Poczekaj na następny przedział
    // ----------------------------------------

    if (
        static_cast<int32_t>(
            now - nextIntervalTime
        ) < 0
    )
    {
        return;
    }

    // ----------------------------------------
    // 3. Sprawdź koniec trajektorii
    // ----------------------------------------

    if (currentInterval >= trajectory.size())
    {
        active = false;
        return;
    }

    // ----------------------------------------
    // 4. Pobierz nowy przedział
    // ----------------------------------------

    int stepValue = trajectory[currentInterval];

    currentInterval++;

    // Każdy przedział otrzymuje pełny czas.
    // W razie opóźnienia cała trajektoria
    // może trwać dłużej.
    nextIntervalTime = now + intervalDurationUs;

    stepsRemaining = std::abs(stepValue);

    if (stepsRemaining == 0)
    {
        return;
    }

    // ----------------------------------------
    // 5. Ustaw kierunek
    // ----------------------------------------

    motorDriver.setDirection(stepValue < 0);

    // ----------------------------------------
    // 6. Wyznacz odstęp między krokami
    // ----------------------------------------

    stepPeriodUs =
        intervalDurationUs /
        static_cast<uint32_t>(stepsRemaining);

    if (stepPeriodUs == 0)
    {
        stepPeriodUs = 1;
    }

    nextStepTime = micros() + stepPeriodUs;
}
