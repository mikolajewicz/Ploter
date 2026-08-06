#ifndef SERIAL_COMMAND_HANDLER_H
#define SERIAL_COMMAND_HANDLER_H

#include <Arduino.h>

class MotorDriver;
class TMC2209Stepper;

class SerialCommandHandler {
public:
    SerialCommandHandler(MotorDriver& motor, TMC2209Stepper& tmc);

    void printHelp();
    void printStatus();
    bool parseLongArgument(const String& argument, long& result);
    void handleSerialCommand(String line);
    void readSerialCommands();

private:
    MotorDriver& motor_;
    TMC2209Stepper& tmc_;
    String serialCommand_;
};

#endif
