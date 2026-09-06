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
    TMC2209Stepper& tmc1,
    MotionExecutor& motionExecutor1,
    TrajectoryGenerator& trajectoryGenerator1,

    MotorDriver& motor2,
    TMC2209Stepper& tmc2,
    MotionExecutor& motionExecutor2,
    TrajectoryGenerator& trajectoryGenerator2
);

    void printHelp();
    void printStatus();
    bool parseLongArgument(const String& argument, long& result);
    bool parseDoubleArgument(const String& argument, double& result);
    void handleSerialCommand(String line);
    void readSerialCommands();

    bool trapeze(
    uint8_t motor,
    double distance,
    double time,
    double acceleration,
    double timeStep
);

    bool cosine(
    uint8_t motor,
    double amplitude,
    double frequency,
    double duration,
    double timeStep
);
private:
    MotorDriver* motors_[2];
    TMC2209Stepper* tmcs_[2];
    MotionExecutor* motionExecutors_[2];
    TrajectoryGenerator* trajectoryGenerators_[2];

    uint8_t selectedMotor_ = 0;

    String serialCommand_;
};

#endif
