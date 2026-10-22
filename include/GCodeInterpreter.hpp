#pragma once

#include <Arduino.h>
#include <GCodeParser.h>
#include "MotionManager.hpp"

class MotionManager;

class GCodeInterpreter
{
public:
    void processCommand(
        const String& line,
        MotionManager& motionManager
    );

    void setPosition(
        double x,
        double y
    );
private:
    GCodeParser parser_;

    // G-code: mm/min
    double feedrate_ = 1200.0;

    bool absoluteMode_ = true;

    double currentX_ = 0.0;
    double currentY_ = 0.0;
};