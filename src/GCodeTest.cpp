
#include <Arduino.h>
#include <GCodeParser.h>

extern void sendResponse(const char* message);

void testGCode()
{
    sendResponse("GCODE TEST START");
    GCodeParser gcode;

    char command[] = "G1 X100 Y50 F1200";
    char response[128];

    sendResponse("=== GCODE TEST ===");
    sendResponse(command);

    gcode.ParseLine(command);

    const char parameters[] = {'G', 'X', 'Y', 'F'};

    for (char parameter : parameters)
    {
        if (gcode.HasWord(parameter))
        {
            snprintf(
                response,
                sizeof(response),
                "%c = %.3f",
                parameter,
                gcode.GetWordValue(parameter)
            );

            sendResponse(response);
        }
    }

    sendResponse("=== END ===");
}
