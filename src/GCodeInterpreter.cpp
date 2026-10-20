#include "GCodeInterpreter.hpp"
#include "MotionManager.hpp"

extern void sendResponse(const char* message);

void GCodeInterpreter::processCommand(
    const String& line,
    MotionManager& motionManager
)
{
    char buffer[112];
    line.toCharArray(buffer, sizeof(buffer));

    parser_.ParseLine(buffer);

    char response[160];

    int motionCode = -1;

    // --------------------------------------------------
    // G
    // --------------------------------------------------

    if (parser_.HasWord('G'))
    {
        int g = static_cast<int>(
            parser_.GetWordValue('G')
        );

        // G90 - współrzędne absolutne
        if (g == 90)
        {
            absoluteMode_ = true;

            sendResponse(
                "GCODE: G90 absolute mode"
            );

            return;
        }

        // G91 - współrzędne względne
        if (g == 91)
        {
            absoluteMode_ = false;

            sendResponse(
                "GCODE: G91 relative mode"
            );

            return;
        }

        // G21 - milimetry
        if (g == 21)
        {
            sendResponse(
                "GCODE: G21 millimeters"
            );

            return;
        }

        // Obsługiwane ruchy:
        //
        // G0 - linia
        // G1 - linia
        // G2 - łuk CW
        // G3 - łuk CCW

        if (
            g != 0 &&
            g != 1 &&
            g != 2 &&
            g != 3
        )
        {
            sendResponse(
                "GCODE: unsupported G"
            );

            return;
        }

        motionCode = g;
    }

    // --------------------------------------------------
    // Feedrate
    // --------------------------------------------------

    if (parser_.HasWord('F'))
    {
        double feed =
            parser_.GetWordValue('F');

        if (feed <= 0.0)
        {
            sendResponse(
                "GCODE: invalid feedrate"
            );

            return;
        }

        feedrate_ = feed;
    }

    // Samo F jest dozwolone
    if (motionCode == -1)
    {
        sendResponse("GCODE: OK");
        return;
    }

    // --------------------------------------------------
    // Aktualna pozycja plotera
    // --------------------------------------------------

    if (!motionManager.calculateCurrentPosition())
    {
        sendResponse(
            "GCODE: cannot calculate current position"
        );

        return;
    }

    double currentX =
        motionManager.getCurrentX();

    double currentY =
        motionManager.getCurrentY();

    double targetX = currentX;
    double targetY = currentY;

    // --------------------------------------------------
    // X
    // --------------------------------------------------

    if (parser_.HasWord('X'))
    {
        double x =
            parser_.GetWordValue('X');

        targetX =
            absoluteMode_
            ? x
            : currentX + x;
    }

    // --------------------------------------------------
    // Y
    // --------------------------------------------------

    if (parser_.HasWord('Y'))
    {
        double y =
            parser_.GetWordValue('Y');

        targetY =
            absoluteMode_
            ? y
            : currentY + y;
    }

    // --------------------------------------------------
    // Prędkość
    //
    // G-code F = mm/min
    // MotionManager = mm/s
    // --------------------------------------------------

    double speed =
        feedrate_ / 60.0;

    if (speed <= 0.0)
    {
        sendResponse(
            "GCODE: invalid speed"
        );

        return;
    }

    // ==================================================
    // G0 / G1
    // ==================================================

    if (
        motionCode == 0 ||
        motionCode == 1
    )
    {
        bool hasX =
            parser_.HasWord('X');

        bool hasY =
            parser_.HasWord('Y');

        if (!hasX && !hasY)
        {
            sendResponse(
                "GCODE: no movement"
            );

            return;
        }

        bool ok =
            motionManager.planLine(
                currentX,
                currentY,
                targetX,
                targetY,
                speed,
                0.002
            );

        if (!ok)
        {
            sendResponse(
                "GCODE: motion queue full"
            );

            return;
        }

        snprintf(
            response,
            sizeof(response),
            "GCODE LINE: G%d %.3f %.3f -> %.3f %.3f speed=%.3f",
            motionCode,
            currentX,
            currentY,
            targetX,
            targetY,
            speed
        );

        sendResponse(response);

        x_ = targetX;
        y_ = targetY;

        return;
    }

    // ==================================================
    // G2 / G3
    // ==================================================

    if (
        motionCode == 2 ||
        motionCode == 3
    )
    {
        // Na razie wymagamy I oraz J.
        //
        // I = przesunięcie środka względem start X
        // J = przesunięcie środka względem start Y

        if (
            !parser_.HasWord('I') ||
            !parser_.HasWord('J')
        )
        {
            sendResponse(
                "GCODE: G2/G3 requires I and J"
            );

            return;
        }

        double i =
            parser_.GetWordValue('I');

        double j =
            parser_.GetWordValue('J');

        double centerX =
            currentX + i;

        double centerY =
            currentY + j;

        bool clockwise =
            motionCode == 2;

        bool ok =
            motionManager.planArc(
                currentX,
                currentY,
                targetX,
                targetY,
                centerX,
                centerY,
                clockwise,
                speed,
                0.002
            );

        if (!ok)
        {
            sendResponse(
                "GCODE: arc queue full"
            );

            return;
        }

        snprintf(
            response,
            sizeof(response),
            "GCODE ARC: G%d %.3f %.3f -> %.3f %.3f center=%.3f %.3f speed=%.3f",
            motionCode,
            currentX,
            currentY,
            targetX,
            targetY,
            centerX,
            centerY,
            speed
        );

        sendResponse(response);

        x_ = targetX;
        y_ = targetY;

        return;
    }
}