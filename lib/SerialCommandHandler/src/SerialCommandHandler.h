#ifndef SERIAL_COMMAND_HANDLER_H
#define SERIAL_COMMAND_HANDLER_H

#include <Arduino.h>

class MotorDriver;
class TMC2209Stepper;
class MotionExecutor;
class TrajectoryGenerator;

class SerialCommandHandler {
public:
    SerialCommandHandler(
        MotorDriver& motor,
        TMC2209Stepper& tmc,
        MotionExecutor* motionExecutor = nullptr,
        TrajectoryGenerator* trajectoryGenerator = nullptr
    );

    void printHelp();
    void printStatus();
    bool parseLongArgument(const String& argument, long& result);
    bool parseDoubleArgument(const String& argument, double& result);
    void handleSerialCommand(String line);
    void readSerialCommands();

private:
    MotorDriver& motor_;
    TMC2209Stepper& tmc_;
    MotionExecutor* motionExecutor_;
    TrajectoryGenerator* trajectoryGenerator_;
    String serialCommand_;
};

#endif
