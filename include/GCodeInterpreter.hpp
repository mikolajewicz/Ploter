#pragma once

#include <Arduino.h>
#include <GCodeParser.h>

class GCodeInterpreter
{
public:
    void processCommand(const String& line);

private:
    GCodeParser parser_;

    double x_ = 0.0;
    double y_ = 0.0;
    double feedrate_ = 1200.0;

    bool absoluteMode_ = true;
};