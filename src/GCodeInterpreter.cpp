
#include "GCodeInterpreter.hpp"

extern void sendResponse(const char* message);

void GCodeInterpreter::processCommand(const String& line)
{
    char buffer[112];
    line.toCharArray(buffer, sizeof(buffer));

    parser_.ParseLine(buffer);

    char response[128];

    if (parser_.HasWord('G'))
    {
        int g = static_cast<int>(
            parser_.GetWordValue('G')
        );

        if (g == 90)
        {
            absoluteMode_ = true;
            sendResponse("GCODE: G90 absolute mode");
            return;
        }

        if (g == 91)
        {
            absoluteMode_ = false;
            sendResponse("GCODE: G91 relative mode");
            return;
        }

        if (g == 21)
        {
            sendResponse("GCODE: G21 millimeters");
            return;
        }

        if (g != 0 && g != 1)
        {
            sendResponse("GCODE: unsupported G");
            return;
        }
    }

    if (parser_.HasWord('F'))
    {
        double feed = parser_.GetWordValue('F');

        if (feed <= 0.0)
        {
            sendResponse("GCODE: invalid feedrate");
            return;
        }

        feedrate_ = feed;
    }

    double targetX = x_;
    double targetY = y_;

    if (parser_.HasWord('X'))
    {
        double x = parser_.GetWordValue('X');
        targetX = absoluteMode_ ? x : x_ + x;
    }

    if (parser_.HasWord('Y'))
    {
        double y = parser_.GetWordValue('Y');
        targetY = absoluteMode_ ? y : y_ + y;
    }

    snprintf(
        response,
        sizeof(response),
        "GCODE: X=%.3f Y=%.3f F=%.1f",
        targetX,
        targetY,
        feedrate_
    );

    sendResponse(response);

    x_ = targetX;
    y_ = targetY;
}
