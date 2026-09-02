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
        MotorDriver& motor1,
        MotorDriver& motor2,
        TMC2209Stepper& tmc1,
        TMC2209Stepper& tmc2,
        MotionExecutor* motionExecutor1 = nullptr,
        MotionExecutor* motionExecutor2 = nullptr,
        TrajectoryGenerator* trajectoryGenerator1 = nullptr,
        TrajectoryGenerator* trajectoryGenerator2 = nullptr
    );

    void printHelp();
    void printStatus();
    bool parseLongArgument(const String& argument, long& result);
    bool parseDoubleArgument(const String& argument, double& result);
    void handleSerialCommand(String line);
    void readSerialCommands();

private:
    MotorDriver* motors_[2];
    TMC2209Stepper* tmcs_[2];
    MotionExecutor* motionExecutors_[2];
    TrajectoryGenerator* trajectoryGenerators_[2];
    String serialCommand_;

    int resolveAxis(const String& value) const;
    MotorDriver& motorForAxis(int axis) const;
    TMC2209Stepper& tmcForAxis(int axis) const;
    MotionExecutor* motionExecutorForAxis(int axis) const;
    TrajectoryGenerator* trajectoryGeneratorForAxis(int axis) const;
    void printStatusForAxis(int axis);
};

#endif
