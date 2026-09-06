#include "SerialCommandHandler.h"

#include <TMCStepper.h>
#include "MotorDriver.hpp"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"

SerialCommandHandler::SerialCommandHandler(
    MotorDriver& motor1,
    TMC2209Stepper& tmc1,
    MotionExecutor& motionExecutor1,
    TrajectoryGenerator& trajectoryGenerator1,

    MotorDriver& motor2,
    TMC2209Stepper& tmc2,
    MotionExecutor& motionExecutor2,
    TrajectoryGenerator& trajectoryGenerator2
)
    : motors_{&motor1, &motor2},
      tmcs_{&tmc1, &tmc2},
      motionExecutors_{&motionExecutor1, &motionExecutor2},
      trajectoryGenerators_{&trajectoryGenerator1, &trajectoryGenerator2} {
}

void SerialCommandHandler::printHelp() {
    Serial.println();
    Serial.println("Dostepne komendy:");
    Serial.println("  enable");
    Serial.println("  disable");
    Serial.println("  start");
    Serial.println("  stop");
    Serial.println("  speed <kroki/s>");
    Serial.println("  dir 0");
    Serial.println("  dir 1");
    Serial.println("  rms <mA>");
    Serial.println("  microsteps <wartosc>");
    Serial.println("  mode stealth");
    Serial.println("  mode spread");
    Serial.println("  cosine <amplitude> <frequency> <duration> <dt>");
    Serial.println("  <motor> trapeze <distance> <time> <accelTime> <dt>");
    Serial.println("  status");
    Serial.println("  help");
    Serial.println();

    Serial.println("Przyklady:");
    Serial.println("  speed 1000");
    Serial.println("  dir 0");
    Serial.println("  rms 700");
    Serial.println("  microsteps 16");
    Serial.println("  cosine 100 2 10 0.01");
    Serial.println("  m2 trapeze -75 2 0.5 0.01");
    Serial.println();
}

void SerialCommandHandler::printStatus() {
    MotorDriver& motor_ =
        *motors_[selectedMotor_];

    TMC2209Stepper& tmc_ =
        *tmcs_[selectedMotor_];

    Serial.println();
    Serial.println("----- STATUS -----");

    Serial.print("Enabled: ");
    Serial.println(
        motor_.isEnabled() ? "yes" : "no"
    );

    Serial.print("Running: ");
    Serial.println(
        motor_.isRunning() ? "yes" : "no"
    );

    Serial.print("Speed: ");
    Serial.print(motor_.getSpeed());
    Serial.println(" steps/s");

    Serial.print("Direction: ");
    Serial.println(
        motor_.getDirection() ? "1" : "0"
    );

    Serial.print("IFCNT: ");
    Serial.println(tmc_.IFCNT());

    Serial.print("TMC version: ");
    Serial.println(tmc_.version());

    Serial.print("DRV_STATUS: 0x");
    Serial.println(tmc_.DRV_STATUS(), HEX);

    Serial.println("------------------");
    Serial.println();
}

bool SerialCommandHandler::parseLongArgument(
    const String& argument,
    long& result
) {
    if (argument.length() == 0) {
        return false;
    }

    char* endPointer = nullptr;

    result = strtol(
        argument.c_str(),
        &endPointer,
        10
    );

    return (
        endPointer != argument.c_str() &&
        *endPointer == '\0'
    );
}

bool SerialCommandHandler::parseDoubleArgument(
    const String& argument,
    double& result
) {
    if (argument.length() == 0) {
        return false;
    }

    char* endPointer = nullptr;

    result = strtod(
        argument.c_str(),
        &endPointer
    );

    return (
        endPointer != argument.c_str() &&
        *endPointer == '\0'
    );
}

void SerialCommandHandler::handleSerialCommand(String line) {
    line.trim();

    if (line.length() == 0) {
        return;
    }

    

    String globalCommand = line;
    globalCommand.toLowerCase();

    if (globalCommand == "stop") {
        stopAll();
        Serial.println("ALL STOPPED");
        return;
    }

    // ------------------------------------------------
    // wybor silnika
    // ------------------------------------------------

    int motorSeparator = line.indexOf(' ');

    if (motorSeparator < 0) {
        Serial.println(
            "Uzycie: m1 <komenda> lub m2 <komenda>"
        );
        return;
    }

    String motorName =
        line.substring(0, motorSeparator);

    motorName.toLowerCase();

    if (motorName == "m1") {
        selectedMotor_ = 0;
    }
    else if (motorName == "m2") {
        selectedMotor_ = 1;
    }
    else {
        Serial.println(
            "Nieznany silnik. Uzyj m1 lub m2"
        );
        return;
    }

    line = line.substring(motorSeparator + 1);
    line.trim();

    if (line.length() == 0) {
        return;
    }

    // Te nazwy sa takie same jak poprzednie pola klasy,
    // wiec dalsza czesc handlera nie wymaga zmian.
    MotorDriver& motor_ =
        *motors_[selectedMotor_];

    TMC2209Stepper& tmc_ =
        *tmcs_[selectedMotor_];

    MotionExecutor* motionExecutor_ =
        motionExecutors_[selectedMotor_];

    TrajectoryGenerator* trajectoryGenerator_ =
        trajectoryGenerators_[selectedMotor_];

    // ------------------------------------------------
    // parsowanie komendy
    // ------------------------------------------------

    int separatorPosition = line.indexOf(' ');

    String command;
    String argument;

    if (separatorPosition < 0) {
        command = line;
        argument = "";
    } else {
        command = line.substring(
            0,
            separatorPosition
        );

        argument = line.substring(
            separatorPosition + 1
        );

        argument.trim();
    }

    command.toLowerCase();

    // ------------------------------------------------
    // enable
    // ------------------------------------------------

    if (command == "enable") {
        motor_.enable();

        Serial.println("Motor enabled");
        return;
    }

    // ------------------------------------------------
    // disable
    // ------------------------------------------------

    if (command == "disable") {
        motor_.disable();

        Serial.println("Motor disabled");
        return;
    }

    // ------------------------------------------------
    // start
    // ------------------------------------------------

    if (command == "start") {
        if (!motor_.isEnabled()) {
            motor_.enable();
        }

        motor_.start();

        if (motor_.getSpeed() == 0) {
            Serial.println(
                "Nie mozna uruchomic: speed = 0"
            );
        } else {
            Serial.println("Motor started");
        }

        return;
    }

    // ------------------------------------------------
    // stop
    // ------------------------------------------------

    if (command == "stop") {
        motor_.stop();

        Serial.println("Motor stopped");
        return;
    }

    // ------------------------------------------------
    // speed
    // ------------------------------------------------

    if (command == "speed") {
        long value = 0;

        if (!parseLongArgument(argument, value)) {
            Serial.println(
                "Uzycie: speed <kroki/s>"
            );
            return;
        }

        if (value < 0) {
            Serial.println(
                "Predkosc nie moze byc ujemna"
            );
            return;
        }

        motor_.setSpeed(
            static_cast<uint32_t>(value)
        );

        Serial.print("Speed set to: ");
        Serial.print(value);
        Serial.println(" steps/s");

        return;
    }

    // ------------------------------------------------
    // dir
    // ------------------------------------------------

    if (command == "dir") {
        if (argument == "0") {
            motor_.setDirection(false);

            Serial.println("Direction set to: 0");
            return;
        }

        if (argument == "1") {
            motor_.setDirection(true);

            Serial.println("Direction set to: 1");
            return;
        }

        Serial.println("Uzycie: dir 0 lub dir 1");
        return;
    }

    // ------------------------------------------------
    // rms
    // ------------------------------------------------

    if (command == "rms") {
        long value = 0;

        if (!parseLongArgument(argument, value)) {
            Serial.println("Uzycie: rms <mA>");
            return;
        }

        if (value <= 0 || value > 3000) {
            Serial.println(
                "Nieprawidlowa wartosc pradu RMS"
            );
            return;
        }

        tmc_.rms_current(
            static_cast<uint16_t>(value)
        );

        Serial.print("RMS current set to: ");
        Serial.print(value);
        Serial.println(" mA");

        return;
    }

    // ------------------------------------------------
    // microsteps
    // ------------------------------------------------

    if (command == "microsteps") {
        long value = 0;

        if (!parseLongArgument(argument, value)) {
            Serial.println(
                "Uzycie: microsteps <wartosc>"
            );
            return;
        }

        switch (value) {
            case 0:
            case 2:
            case 4:
            case 8:
            case 16:
            case 32:
            case 64:
            case 128:
            case 256:
                tmc_.microsteps(
                    static_cast<uint16_t>(value)
                );

                Serial.print(
                    "Microsteps set to: "
                );
                Serial.println(value);
                return;

            default:
                Serial.println(
                    "Dozwolone: 0, 2, 4, 8, 16, "
                    "32, 64, 128, 256"
                );
                return;
        }
    }

    // ------------------------------------------------
    // mode
    // ------------------------------------------------

    if (command == "mode") {
        String mode = argument;
        mode.toLowerCase();

        if (mode == "stealth") {
            tmc_.en_spreadCycle(false);
            tmc_.pwm_autoscale(true);

            Serial.println(
                "Mode set to StealthChop"
            );
            return;
        }

        if (mode == "spread") {
            tmc_.en_spreadCycle(true);

            Serial.println(
                "Mode set to SpreadCycle"
            );
            return;
        }

        Serial.println(
            "Uzycie: mode stealth lub mode spread"
        );
        return;
    }

    // ------------------------------------------------
    // trapeze
    // ------------------------------------------------

    if (command == "trapeze") {
    String params[4];
    String remaining = argument;
    remaining.trim();

    for (int i = 0; i < 4; ++i) {
        int separator = remaining.indexOf(' ');

        if (separator < 0) {
            params[i] = remaining;
            remaining = "";
        } else {
            params[i] = remaining.substring(0, separator);
            remaining = remaining.substring(separator + 1);
            remaining.trim();
        }

        params[i].trim();

        if (params[i].length() == 0) {
            Serial.println(
                "Uzycie: trapeze <distance> <time> <acceleration> <dt>"
            );
            return;
        }
    }

    double distance = 0.0;
    double totalTime = 0.0;
    double acceleration = 0.0;
    double timeStep = 0.0;

    if (!parseDoubleArgument(params[0], distance) ||
        !parseDoubleArgument(params[1], totalTime) ||
        !parseDoubleArgument(params[2], acceleration) ||
        !parseDoubleArgument(params[3], timeStep)) {

        Serial.println(
            "Uzycie: trapeze <distance> <time> <acceleration> <dt>"
        );
        return;
    }

    if (totalTime <= 0.0) {
        Serial.println("Czas trwania musi byc > 0");
        return;
    }

    if (acceleration <= 0.0) {
        Serial.println("Przyspieszenie musi byc > 0");
        return;
    }

    if (timeStep <= 0.0) {
        Serial.println("Krok czasowy musi byc > 0");
        return;
    }

    if (!trapeze(
            selectedMotor_,
            distance,
            totalTime,
            acceleration,
            timeStep
        )) {

        Serial.println(
            "Nie udalo sie uruchomic profilu trapezoidalnego"
        );
        return;
    }

    Serial.print("Trapeze configured: distance=");
    Serial.print(distance);
    Serial.print(", time=");
    Serial.print(totalTime);
    Serial.print("s, acceleration=");
    Serial.print(acceleration);
    Serial.print(", dt=");
    Serial.print(timeStep);
    Serial.println("s");

    return;
}

    // ------------------------------------------------
    // sine
    // ------------------------------------------------

    if (command == "sine") {
        if (motionExecutor_ == nullptr || trajectoryGenerator_ == nullptr) {
            Serial.println(
                "Komenda sine jest niezainicjalizowana."
            );
            return;
        }

        String params[4];
        String remaining = argument;
        remaining.trim();

        for (int i = 0; i < 4; ++i) {
            int separator = remaining.indexOf(' ');

            if (separator < 0) {
                params[i] = remaining;
                remaining = "";
            } else {
                params[i] = remaining.substring(0, separator);
                remaining = remaining.substring(separator + 1);
                remaining.trim();
            }

            params[i].trim();

            if (params[i].length() == 0) {
                Serial.println(
                    "Uzycie: sine <amplitude> <frequency> <dt>"
                );
                return;
            }
        }

        double amplitude = 0.0;
        double frequency = 0.0;
        double timeStep = 0.0;
        double duration = 0.0;

        if (!parseDoubleArgument(params[0], amplitude) ||
            !parseDoubleArgument(params[1], frequency) ||
            !parseDoubleArgument(params[2], duration) ||
            !parseDoubleArgument(params[3], timeStep)) {
            Serial.println(
                "Uzycie: sine <amplitude> <frequency> <duration> <dt>"
            );
            return;
        }

        if (amplitude < 0.0) {
            Serial.println("Amplituda nie moze byc ujemna");
            return;
        }

        if (frequency <= 0.0) {
            Serial.println("Czestotliwosc musi byc > 0");
            return;
        }

        if (timeStep <= 0.0) {
            Serial.println("Krok czasowy musi byc > 0");
            return;
        }

        if (duration <= 0.0) {
            Serial.println("Czas trwania musi byc > 0");
            return;
        }

        if (!cosine(
            selectedMotor_,
            amplitude,
            frequency,
            duration,
            timeStep
        )) {
            Serial.println(
                "Nie udalo sie uruchomic profilu sinusoidalnego"
            );
            return;
        }

        Serial.print("Sine configured: amplitude=");
        Serial.print(amplitude);
        Serial.print(", frequency=");
        Serial.print(frequency);
        Serial.print("Hz, duration=");
        Serial.print(duration);
        Serial.print("s, dt=");
        Serial.print(timeStep);
        Serial.println("s");

        return;
    }

    // ------------------------------------------------
    // status
    // ------------------------------------------------

    if (command == "status") {
        printStatus();
        return;
    }

    // ------------------------------------------------
    // help
    // ------------------------------------------------

    if (command == "help") {
        printHelp();
        return;
    }

    Serial.println(
        "Nieznana komenda. Wpisz: help"
    );
}

void SerialCommandHandler::readSerialCommands() {
    while (Serial.available() > 0) {
        char character = Serial.read();

        if (character == '\r') {
            continue;
        }

        if (character == '\n') {
            handleSerialCommand(serialCommand_);
            serialCommand_ = "";
            continue;
        }

        // Ochrona przed nieograniczonym wzrostem String.
        if (serialCommand_.length() < 100) {
            serialCommand_ += character;
        } else {
            serialCommand_ = "";

            Serial.println(
                "Komenda jest zbyt dluga"
            );
        }
    }
}

bool SerialCommandHandler::trapeze(
    uint8_t motor,
    double distance,
    double time,
    double acceleration,
    double timeStep
) {
    uint8_t motorIndex = motor;

    if (motor == 1) {
        motorIndex = 0;
    } else if (motor == 2) {
        motorIndex = 1;
    } else if (motor != 0) {
        return false;
    }

    MotionExecutor* motionExecutor =
        motionExecutors_[motorIndex];

    TrajectoryGenerator* trajectoryGenerator =
        trajectoryGenerators_[motorIndex];

    MotorDriver* motorDriver =
        motors_[motorIndex];

    if (motionExecutor == nullptr ||
        trajectoryGenerator == nullptr) {
        return false;
    }

    if (time <= 0.0 ||
        acceleration <= 0.0 ||
        timeStep <= 0.0) {
        return false;
    }

    std::vector<int> stepTrajectory;

    if (!trajectoryGenerator->trapezoidalProfile(
            distance,
            time,
            acceleration,
            timeStep
        )) {
        return false;
    }

    trajectoryGenerator->convertToSteps(stepTrajectory);

    motionExecutor->setTimeStep(timeStep);
    motionExecutor->start(stepTrajectory, timeStep);

    if (!motorDriver->isEnabled()) {
        motorDriver->enable();
    }

    return true;
}

bool SerialCommandHandler::cosine(
    uint8_t motor,
    double amplitude,
    double frequency,
    double duration,
    double timeStep
) {
    uint8_t motorIndex = motor;

    if (motor == 1) {
        motorIndex = 0;
    } else if (motor == 2) {
        motorIndex = 1;
    } else if (motor != 0) {
        return false;
    }

    MotionExecutor* motionExecutor =
        motionExecutors_[motorIndex];

    TrajectoryGenerator* trajectoryGenerator =
        trajectoryGenerators_[motorIndex];

    MotorDriver* motorDriver =
        motors_[motorIndex];

    if (motionExecutor == nullptr ||
        trajectoryGenerator == nullptr) {
        return false;
    }

    if (amplitude < 0.0 ||
        frequency <= 0.0 ||
        duration <= 0.0 ||
        timeStep <= 0.0) {
        return false;
    }

    std::vector<int> stepTrajectory;

    trajectoryGenerator->cosinusoidalTrajectory(
        amplitude,
        frequency,
        duration,
        timeStep
    );
    trajectoryGenerator->convertToSteps(stepTrajectory);

    motionExecutor->setTimeStep(timeStep);
    motionExecutor->start(stepTrajectory, timeStep);

    if (!motorDriver->isEnabled()) {
        motorDriver->enable();
    }

    return true;
}

void SerialCommandHandler::stopAll()
{
    motionExecutors_[0]->stop();
    motionExecutors_[1]->stop();

    motors_[0]->stop();
    motors_[1]->stop();

    stopped_ = true;
}