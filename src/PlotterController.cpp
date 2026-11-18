#include "PlotterController.hpp"

#include "CommunicationManager.hpp"
#include "MotorDriver.hpp"
#include "MotionManager.hpp"
#include "HomingSequence.hpp"
#include "PenController.hpp"
#include "GCodeProgram.hpp"
#include "SerialCommandHandler.h"

#include <cstring>

PlotterController::PlotterController(
    MotorDriver& motor1,
    MotorDriver& motor2,
    MotionManager& motionManager,
    HomingSequence& homingSequence,
    PenController& pen,
    GCodeProgram& gcodeProgram,
    SerialCommandHandler& commandHandler
)
    : motor1_(motor1),
      motor2_(motor2),
      motionManager_(motionManager),
      homingSequence_(homingSequence),
      pen_(pen),
      gcodeProgram_(gcodeProgram),
      commandHandler_(commandHandler)
{
}

void PlotterController::processCommands()
{
    TcpCommand command = {};

    // Zachowujemy dotychczasowa kolejnosc: jedna komenda ogolna...
    if (tcpCommands != nullptr &&
        xQueueReceive(tcpCommands, &command, 0) == pdTRUE)
    {
        if (strcmp(command.text, "GCODE_RUN") == 0)
        {
            if (!homingSequence_.isReady())
            {
                sendResponse("ERR GCODE HOME");
            }
            else if (!gcodeProgram_.start())
            {
                sendResponse("ERR GCODE RUN");
            }
        }
        else if (strcmp(command.text, "stop") == 0)
        {
            stop();
        }
        else if (strcmp(command.text, "DISABLE") == 0)
        {
            stop();

            motor1_.disable();
            motor2_.disable();
            homingSequence_.reset();
            pen_.up();

            sendResponse("MOTORS DISABLED");
        }
        else if (strcmp(command.text, "ENABLE") == 0)
        {
            motor1_.enable();
            motor2_.enable();

            sendResponse("MOTORS ENABLED");
        }
        else
        {
            commandHandler_.processCommand(command.text);
        }
    }

    // ...i jedna, zawsze najnowsza pozycja tabletu.
    if (tabletCommands != nullptr &&
        xQueueReceive(tabletCommands, &command, 0) == pdTRUE)
    {
        if (!gcodeProgram_.isRunning())
        {
            commandHandler_.processCommand(command.text);
        }
    }
}

void PlotterController::stop()
{
    gcodeProgram_.stop();
    motionManager_.clearLiveTarget();
    motionManager_.clearPreparedMotion();
    homingSequence_.abort();
    commandHandler_.processCommand("stop");
}
