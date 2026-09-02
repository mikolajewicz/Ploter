#include "MotorDriver.hpp"

MotorDriver::MotorDriver(
    int stepPin,
    int dirPin,
    int enablePin
)
    : stepPin(stepPin),
      dirPin(dirPin),
      enablePin(enablePin),
      speedStepsPerSecond(0),
      halfPeriodMicroseconds(0),
      previousToggleTime(0),
      direction(true),
      enabled(false),
      running(false),
      stepState(false)
{
}

void MotorDriver::begin() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, direction ? HIGH : LOW);

    if (enablePin != -1) {
        pinMode(enablePin, OUTPUT);

        // TMC2209: EN jest aktywne stanem LOW.
        digitalWrite(enablePin, HIGH);
    }

    enabled = false;
    running = false;
    stepState = false;
}

void MotorDriver::enable() {
    if (enablePin != -1) {
        digitalWrite(enablePin, LOW);
    }

    enabled = true;
}

void MotorDriver::disable() {
    stop();

    if (enablePin != -1) {
        digitalWrite(enablePin, HIGH);
    }

    enabled = false;
}

void MotorDriver::start() {
    if (!enabled || speedStepsPerSecond == 0) {
        return;
    }

    previousToggleTime = micros();
    running = true;
}

void MotorDriver::stop() {
    running = false;
    stepState = false;

    digitalWrite(stepPin, LOW);
}

void MotorDriver::setDirection(bool newDirection) {
    bool wasRunning = running;

    stop();

    direction = newDirection;
    digitalWrite(dirPin, direction ? HIGH : LOW);

    // Krótki czas ustalenia sygnału DIR przed kolejnym STEP.
    delayMicroseconds(5);

    if (wasRunning && enabled && speedStepsPerSecond > 0) {
        start();
    }
}

void MotorDriver::setSpeed(uint32_t stepsPerSecond) {
    speedStepsPerSecond = stepsPerSecond;

    if (stepsPerSecond == 0) {
        halfPeriodMicroseconds = 0;
        stop();
        return;
    }

    // Jeden krok wymaga pełnego impulsu HIGH + LOW.
    halfPeriodMicroseconds =
        1000000UL / (2UL * stepsPerSecond);

    // Ochrona przed wynikiem 0 przy bardzo dużej częstotliwości.
    if (halfPeriodMicroseconds == 0) {
        halfPeriodMicroseconds = 1;
    }

    if (enabled) {
        start();
    }
}

void MotorDriver::run() {
    if (
        !enabled ||
        !running ||
        halfPeriodMicroseconds == 0
    ) {
        return;
    }

    uint32_t currentTime = micros();

    if (
        static_cast<uint32_t>(
            currentTime - previousToggleTime
        ) >= halfPeriodMicroseconds
    ) {
        previousToggleTime = currentTime;

        stepState = !stepState;

        digitalWrite(
            stepPin,
            stepState ? HIGH : LOW
        );
    }
}

bool MotorDriver::isEnabled() const {
    return enabled;
}

bool MotorDriver::isRunning() const {
    return running;
}

uint32_t MotorDriver::getSpeed() const {
    return speedStepsPerSecond;
}

bool MotorDriver::getDirection() const {
    return direction;
}

void MotorDriver::step() {
    if (!enabled) {
        return;
    }

    // Wygeneruj impuls STEP.
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1); // Krótki czas trwania impulsu
    digitalWrite(stepPin, LOW);
}