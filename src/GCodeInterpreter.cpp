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


    normalizedLine.trim();

    // --------------------------------------------------
    // MVS - ruch liniowy przez A2B
    // Bez korzystania z GCodeParser
    // --------------------------------------------------

    bool isMvs =
        normalizedLine == "MVS" ||
        normalizedLine.startsWith("MVS ") ||
        normalizedLine.startsWith("MVS\t");

    if (isMvs)
    {
        double startX = currentX_;
        double startY = currentY_;

        double targetX = startX;
        double targetY = startY;
        double duration = 0.0;

        bool hasX = false;
        bool hasY = false;
        bool hasT = false;

        // Odczyt parametrów X, Y i T
        const char* p = normalizedLine.c_str() + 3;

        while (*p)
        {
            while (*p == ' ' || *p == '\t')
            {
                ++p;
            }

            if (*p == '\0')
            {
                break;
            }

            char word = *p++;

            if (word != 'X' && word != 'Y' && word != 'T')
            {
                sendResponse("ERR MVS unknown parameter");
                return;
            }

            char* end = nullptr;
            double value = strtod(p, &end);

            if (end == p || !std::isfinite(value))
            {
                sendResponse("ERR MVS invalid parameter");
                return;
            }

            p = end;

            if (word == 'X')
            {
                if (hasX)
                {
                    sendResponse("ERR MVS duplicate X");
                    return;
                }

                hasX = true;

                targetX = absoluteMode_
                    ? value
                    : startX + value;
            }
            else if (word == 'Y')
            {
                if (hasY)
                {
                    sendResponse("ERR MVS duplicate Y");
                    return;
                }

                hasY = true;

                targetY = absoluteMode_
                    ? value
                    : startY + value;
            }
            else
            {
                if (hasT)
                {
                    sendResponse("ERR MVS duplicate T");
                    return;
                }

                hasT = true;
                duration = value;
            }
        }

        // Kontrola parametrów
        if (!hasX && !hasY)
        {
            sendResponse("ERR MVS missing X/Y");
            return;
        }

        if (!hasT || duration < 0.05)
        {
            sendResponse("ERR MVS invalid T");
            return;
        }

        // Kontrola obszaru roboczego
        if (!std::isfinite(targetX) ||
            !std::isfinite(targetY) ||
            !isInsideWorkspace(targetX, targetY))
        {
            sendResponse("ERR MVS outside workspace");
            return;
        }

        // Około 10 ms między próbkami.
        // Maksymalnie 600 punktów trajektorii.
        double segments = std::ceil(duration / 0.01);

        if (segments > GCODE_MAX_LINE_POINTS - 1)
        {
            segments = GCODE_MAX_LINE_POINTS - 1;
        }

        double timeStep = duration / segments;

        // Istniejący planer A2B
        bool ok = motionManager.planA2B(
            startX,
            startY,
            targetX,
            targetY,
            duration,
            timeStep
        );

        if (!ok)
        {
            sendResponse("ERR MVS queue full");
            return;
        }

        // Aktualizacja pozycji logicznej G-code
        currentX_ = targetX;
        currentY_ = targetY;

        char response[128];

        snprintf(
            response,
            sizeof(response),
            "MVS %.3f %.3f -> %.3f %.3f T=%.3f",
            startX,
            startY,
            targetX,
            targetY,
            duration
        );

        sendResponse(response);
        return;
    }


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

        
        double radius = std::hypot(
            startX - centerX,
            startY - centerY
        );

        double endRadius = std::hypot(
            targetX - centerX,
            targetY - centerY
        );

        if (!std::isfinite(radius) ||
            !std::isfinite(endRadius) ||
            radius < 0.001 ||
            std::abs(radius - endRadius) > 0.5)
        {
            sendResponse("ERR GCODE invalid arc radius");
            return;
        }

        // Kąt początkowy i końcowy
        double startAngle = std::atan2(
            startY - centerY,
            startX - centerX
        );

        double endAngle = std::atan2(
            targetY - centerY,
            targetX - centerX
        );

        // Dodatnia wartość kąta zakreślanego łuku
        double sweep = clockwise
            ? startAngle - endAngle
            : endAngle - startAngle;

        sweep = std::fmod(sweep, 2.0 * M_PI);

        if (sweep <= 0.0)
        {
            sweep += 2.0 * M_PI;
        }

        // Długość łuku w milimetrach
        double arcLength = radius * sweep;

        // Ograniczamy liczbę punktów trajektorii
        double timeStep = calculateGCodeLineTimeStep(
            speed,
            arcLength
        );

        bool ok = motionManager.planArc(
            startX,
            startY,
            targetX,
            targetY,
            centerX,
            centerY,
            clockwise,
            speed,
            timeStep
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