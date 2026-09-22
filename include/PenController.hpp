#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

class PenController
{
public:
    PenController(
        uint8_t pin,
        int upAngle,
        int downAngle
    );

    void begin();

    void up();
    void down();

    void setPressure(double pressure);

    bool isDown() const;

private:
    Servo servo;

    uint8_t pin;

    int upAngle;
    int downAngle;

    bool penDown = false;

    static constexpr double PRESSURE_THRESHOLD = 0.01;
};