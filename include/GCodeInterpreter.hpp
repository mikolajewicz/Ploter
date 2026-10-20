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

private:
    GCodeParser parser_;

    double x_ = 0.0;
    double y_ = 0.0;

    // G-code: mm/min
    double feedrate_ = 1200.0;

    bool absoluteMode_ = true;
};