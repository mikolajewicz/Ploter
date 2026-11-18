#pragma once

class MotorDriver;
class MotionManager;
class HomingSequence;
class PenController;
class GCodeProgram;
class SerialCommandHandler;

// Wykonuje komendy otrzymane przez USB i TCP.
// Nie odbiera danych z portow - to zadanie CommunicationManager.
class PlotterController
{
public:
    PlotterController(
        MotorDriver& motor1,
        MotorDriver& motor2,
        MotionManager& motionManager,
        HomingSequence& homingSequence,
        PenController& pen,
        GCodeProgram& gcodeProgram,
        SerialCommandHandler& commandHandler
    );

    void processCommands();
    void stop();

private:
    MotorDriver& motor1_;
    MotorDriver& motor2_;
    MotionManager& motionManager_;
    HomingSequence& homingSequence_;
    PenController& pen_;
    GCodeProgram& gcodeProgram_;
    SerialCommandHandler& commandHandler_;
};
