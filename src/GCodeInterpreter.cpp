#include "GCodeInterpreter.hpp"
#include "MotionManager.hpp"
#include <cstdlib>

extern void sendResponse(const char* message);

namespace
{
    constexpr double G0_SPEED_MM_S = 60.0;

    constexpr double GCODE_POINT_DISTANCE_MM = 0.2;

    constexpr double GCODE_MIN_TIME_STEP = 0.002;
    constexpr double GCODE_MAX_TIME_STEP = 0.05;

    constexpr size_t GCODE_MAX_LINE_POINTS = 600;

    double calculateGCodeTimeStep(
        double speed
    )
    {
        if (speed <= 0.0)
        {
            return GCODE_MIN_TIME_STEP;
        }

        double timeStep =
            GCODE_POINT_DISTANCE_MM / speed;

        if (timeStep < GCODE_MIN_TIME_STEP)
        {
            timeStep =
                GCODE_MIN_TIME_STEP;
        }

        if (timeStep > GCODE_MAX_TIME_STEP)
        {
            timeStep =
                GCODE_MAX_TIME_STEP;
        }

        return timeStep;
    }

    double calculateGCodeLineTimeStep(
        double speed,
        double distance
    )
    {
        double timeStep =
            calculateGCodeTimeStep(speed);

        if (
            speed <= 0.0 ||
            distance <= 0.0
        )
        {
            return timeStep;
        }

        double duration =
            distance / speed;

        double limitedTimeStep =
            duration /
            static_cast<double>(
                GCODE_MAX_LINE_POINTS - 1
            );

        if (timeStep < limitedTimeStep)
        {
            timeStep = limitedTimeStep;
        }

        return timeStep;
    }
}

void GCodeInterpreter::processCommand(
    const String& line,
    MotionManager& motionManager
)
{
    String normalizedLine = line;
    normalizedLine.toUpperCase();

    char buffer[112];
    normalizedLine.toCharArray(
        buffer,
        sizeof(buffer)
    );

    parser_.ParseLine(buffer);

    char response[160];

    int motionCode = -1;

    // --------------------------------------------------
    // M-CODE
    // --------------------------------------------------

    if (parser_.HasWord('M'))
    {
        int mPosition =
            parser_.FindWord('M');

        int m = static_cast<int>(
            strtol(
                &parser_.line[mPosition + 1],
                nullptr,
                10
            )
        );

        if (m == 3)
        {
            motionManager.penDown();
            return;
        }

        if (m == 5)
        {
            motionManager.penUp();
            return;
        }

        sendResponse(
            "ERR unsupported M-code"
        );

        return;
    }

    // --------------------------------------------------
    // G
    // --------------------------------------------------

    if (parser_.HasWord('G'))
    {
        int gPosition =
            parser_.FindWord('G');

        int g = static_cast<int>(
            strtol(
                &parser_.line[gPosition + 1],
                nullptr,
                10
            )
        );

        char debug[80];

        snprintf(
            debug,
            sizeof(debug),
            "DBG G=%d LINE=%s",
            g,
            line.c_str()
        );

        // sendResponse(debug);

        if (g == 4)
        {
            if (!parser_.HasWord('P'))
            {
                sendResponse(
                    "ERR G4 missing P"
                );
                return;
            }

            double pauseMs =
                parser_.GetWordValue('P');

            if (pauseMs < 0.0)
            {
                sendResponse(
                    "ERR G4 invalid P"
                );
                return;
            }

            motionManager.startPause(
                static_cast<uint32_t>(pauseMs)
            );

            sendResponse(
                "OK G4 pause started"
            );

            return;
        }

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
        if (parser_.HasWord('F'))
        {
            sendResponse(
                "GCODE: OK"
            );
        }
        else
        {
            sendResponse(
                "GCODE: unknown command"
            );
        }

        return;
    }

    // --------------------------------------------------
    // Logiczna pozycja programu G-code
    // --------------------------------------------------

    double startX = currentX_;
    double startY = currentY_;

    double targetX = currentX_;
    double targetY = currentY_;

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
            : startX + x;
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
            : startY + y;
    }

    // --------------------------------------------------
    // Prędkość
    //
    // G-code F = mm/min
    // MotionManager = mm/s
    // --------------------------------------------------

    double speed;

    if (motionCode == 0)
    {
        speed = G0_SPEED_MM_S;
    }
    else
    {
        speed = feedrate_ / 60.0;
    }

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

        double distance =
            std::hypot(
                targetX - startX,
                targetY - startY
            );

        double timeStep =
            calculateGCodeLineTimeStep(
                speed,
                distance
            );

        if (!isInsideWorkspace(targetX, targetY))
        {
            motionManager.penUp();

            sendResponse(
                "ERR GCODE outside workspace"
            );

            return;
        }

        bool ok =
            motionManager.planLine(
                startX,
                startY,
                targetX,
                targetY,
                speed,
                timeStep
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
            startX,
            startY,
            targetX,
            targetY,
            speed
        );

        sendResponse(response);

        currentX_ = targetX;
        currentY_ = targetY;

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
            startX + i;

        double centerY =
            startY + j;

        bool clockwise =
            motionCode == 2;

        bool ok =
            motionManager.planArc(
                startX,
                startY,
                targetX,
                targetY,
                centerX,
                centerY,
                clockwise,
                speed,
                calculateGCodeTimeStep(speed)
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
            startX,
            startY,
            targetX,
            targetY,
            centerX,
            centerY,
            speed
        );

        sendResponse(response);

        currentX_ = targetX;
        currentY_ = targetY;

        return;
    }
}

void GCodeInterpreter::setPosition(
    double x,
    double y
)
{
    currentX_ = x;
    currentY_ = y;
}

bool GCodeInterpreter::isInsideWorkspace(
    double x,
    double y
)
{
    constexpr double X_MAX = 138.5;
    constexpr double Y_MAX = 95.0;

    return
        x >= -X_MAX &&
        x <= X_MAX &&
        y >= -Y_MAX &&
        y <= Y_MAX;
}