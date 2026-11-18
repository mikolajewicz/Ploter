#pragma once

#include <Arduino.h>
#include <TMCStepper.h>

class MotorDriver;
class MotionExecutor;
class TrajectoryGenerator;
class Homing;
class MotionManager;
class PenController;

// Koordynuje bazowanie obu ramion i przejazd do pozycji zerowej.
// Homing.hpp nadal steruje bazowaniem pojedynczego silnika.
class HomingSequence
{
public:
    HomingSequence(
        MotorDriver& motor1,
        TMC2209Stepper& tmc1,
        MotionExecutor& executor1,
        TrajectoryGenerator& generator1,
        Homing& homing1,

        MotorDriver& motor2,
        TMC2209Stepper& tmc2,
        MotionExecutor& executor2,
        TrajectoryGenerator& generator2,
        Homing& homing2,

        MotionManager& motionManager,
        PenController& pen,
        uint32_t stepsPerRevolution,
        uint16_t initialRmsCurrent,
        double timeStep,
        bool bypassHome
    );

    // Wywoływane przy komendzie "home". Odmawia, jeśli HOME już trwa.
    bool request();

    // Uruchamia HOME; wykorzystywane także przy starcie ESP32.
    void start();

    // Wywoływane raz na przebieg loop().
    void update();

    // Anulowanie aktywnej sekwencji HOME (np. STOP).
    void abort();

    // Wyzerowanie stanu po DISABLE.
    void reset();

    bool isActive() const;
    bool isReady() const;
    int state() const;

private:
    int64_t angleToSteps(double angle) const;

    MotorDriver& motor1_;
    TMC2209Stepper& tmc1_;
    MotionExecutor& executor1_;
    TrajectoryGenerator& generator1_;
    Homing& homing1_;

    MotorDriver& motor2_;
    TMC2209Stepper& tmc2_;
    MotionExecutor& executor2_;
    TrajectoryGenerator& generator2_;
    Homing& homing2_;

    MotionManager& motionManager_;
    PenController& pen_;

    uint32_t stepsPerRevolution_;
    uint16_t initialRmsCurrent_;
    double timeStep_;
    bool bypassHome_;

    int state_ = 0;
    bool motor1Homed_ = false;
    bool motor2Homed_ = false;
};
