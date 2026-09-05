#include "SerialCommandHandler.h"

#include <TMCStepper.h>
#include "MotorDriver.hpp"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"

SerialCommandHandler::SerialCommandHandler(
    MotorDriver& motor1,
    MotorDriver& motor2,
    TMC2209Stepper& tmc1,
    TMC2209Stepper& tmc2,
    MotionExecutor* motionExecutor1,
    MotionExecutor* motionExecutor2,
    TrajectoryGenerator* trajectoryGenerator1,
    TrajectoryGenerator* trajectoryGenerator2
)
    : motors_{&motor1, &motor2},
      tmcs_{&tmc1, &tmc2},
      motionExecutors_{motionExecutor1, motionExecutor2},
      trajectoryGenerators_{trajectoryGenerator1, trajectoryGenerator2} {
}

int SerialCommandHandler::resolveAxis(const String& value) const {
    if (value == "1") {
        return 0;
    }

    if (value == "2") {
        return 1;
    }

    return 0;
}

MotorDriver& SerialCommandHandler::motorForAxis(int axis) const {
    return *motors_[axis];
}

TMC2209Stepper& SerialCommandHandler::tmcForAxis(int axis) const {
    return *tmcs_[axis];
}

MotionExecutor* SerialCommandHandler::motionExecutorForAxis(int axis) const {
    return motionExecutors_[axis];
}

TrajectoryGenerator* SerialCommandHandler::trajectoryGeneratorForAxis(int axis) const {
    return trajectoryGenerators_[axis];
}

void SerialCommandHandler::printHelp() {
    Serial.println();
    Serial.println("Dostepne komendy:");
    Serial.println("  [1|2] enable");
    Serial.println("  [1|2] disable");
    Serial.println("  [1|2] start");
    Serial.println("  [1|2] stop");
    Serial.println("  [1|2] speed <kroki/s>");
    Serial.println("  [1|2] dir 0");
    Serial.println("  [1|2] dir 1");
    Serial.println("  [1|2] rms <mA>");
    Serial.println("  [1|2] microsteps <wartosc>");
    Serial.println("  [1|2] mode stealth");
    Serial.println("  [1|2] mode spread");
    Serial.println("  [1|2] sine <amplitude> <frequency> [duration] <dt>");
    Serial.println("  [1|2] status");
    Serial.println("  help");
    Serial.println();

    Serial.println("Przyklady:");
    Serial.println("  1 speed 1000");
    Serial.println("  2 dir 0");
    Serial.println("  1 rms 700");
    Serial.println("  2 sine 100 2 0.01      (duration defaults to 1/frequency)");
    Serial.println();
}

void SerialCommandHandler::printStatusForAxis(int axis) {
    MotorDriver& motor = motorForAxis(axis);
    TMC2209Stepper& tmc = tmcForAxis(axis);

    Serial.print("--- Axis ");
    Serial.print(axis + 1);
    Serial.println(" ---");

    Serial.print("Enabled: ");
    Serial.println(motor.isEnabled() ? "yes" : "no");

    Serial.print("Running: ");
    Serial.println(motor.isRunning() ? "yes" : "no");

    Serial.print("Speed: ");
    Serial.print(motor.getSpeed());
    Serial.println(" steps/s");

    Serial.print("Direction: ");
    Serial.println(motor.getDirection() ? "1" : "0");

    Serial.print("IFCNT: ");
    Serial.println(tmc.IFCNT());

    Serial.print("TMC version: ");
    Serial.println(tmc.version());

    Serial.print("DRV_STATUS: 0x");
    Serial.println(tmc.DRV_STATUS(), HEX);

    Serial.println();
}

void SerialCommandHandler::printStatus() {
    printStatusForAxis(0);
    printStatusForAxis(1);
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

    int firstSeparator = line.indexOf(' ');
    String firstToken;
    String remaining = line;

    if (firstSeparator >= 0) {
        firstToken = line.substring(0, firstSeparator);
        remaining = line.substring(firstSeparator + 1);
        remaining.trim();
    } else {
        firstToken = line;
        remaining = "";
    }

    int axis = 0;
    String commandLine = line;
    commandLine.trim();

    if (firstToken == "1" || firstToken == "2") {
        axis = resolveAxis(firstToken);
        commandLine = remaining;
    }

    int separatorPosition = commandLine.indexOf(' ');

    String command;
    String argument;

    if (separatorPosition < 0) {
        command = commandLine;
        argument = "";
    } else {
        command = commandLine.substring(
            0,
            separatorPosition
        );

        argument = commandLine.substring(
            separatorPosition + 1
        );

        argument.trim();
    }

    command.toLowerCase();
    MotorDriver& motor = motorForAxis(axis);
    TMC2209Stepper& tmc = tmcForAxis(axis);
    MotionExecutor* motionExecutor = motionExecutorForAxis(axis);
    TrajectoryGenerator* trajectoryGenerator = trajectoryGeneratorForAxis(axis);

    // ------------------------------------------------
    // enable
    // ------------------------------------------------

    if (command == "enable") {
        motor.enable();

        Serial.print("Motor axis ");
        Serial.print(axis + 1);
        Serial.println(" enabled");
        return;
    }

    // ------------------------------------------------
    // disable
    // ------------------------------------------------

    if (command == "disable") {
        motor.disable();

        Serial.print("Motor axis ");
        Serial.print(axis + 1);
        Serial.println(" disabled");
        return;
    }

    // ------------------------------------------------
    // start
    // ------------------------------------------------

    if (command == "start") {
        motor.setMotionMode(MotorDriver::MotionMode::ConstantSpeed);

        if (!motor.isEnabled()) {
            motor.enable();
        }

        motor.start();

        if (motor.getSpeed() == 0) {
            Serial.println(
                "Nie mozna uruchomic: speed = 0"
            );
        } else {
            Serial.print("Motor axis ");
            Serial.print(axis + 1);
            Serial.println(" started");
        }

        return;
    }

    // ------------------------------------------------
    // stop
    // ------------------------------------------------

    if (command == "stop") {
        motor.setMotionMode(MotorDriver::MotionMode::ConstantSpeed);
        motor.stop();

        Serial.print("Motor axis ");
        Serial.print(axis + 1);
        Serial.println(" stopped");
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

        motor.setMotionMode(MotorDriver::MotionMode::ConstantSpeed);
        motor.setSpeed(
            static_cast<uint32_t>(value)
        );

        Serial.print("Axis ");
        Serial.print(axis + 1);
        Serial.print(" speed set to: ");
        Serial.print(value);
        Serial.println(" steps/s");

        return;
    }

    // ------------------------------------------------
    // dir
    // ------------------------------------------------

    if (command == "dir") {
        if (argument == "0") {
            motor.setDirection(false);

            Serial.print("Axis ");
            Serial.print(axis + 1);
            Serial.println(" direction set to: 0");
            return;
        }

        if (argument == "1") {
            motor.setDirection(true);

            Serial.print("Axis ");
            Serial.print(axis + 1);
            Serial.println(" direction set to: 1");
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

        tmc.rms_current(
            static_cast<uint16_t>(value)
        );

        Serial.print("Axis ");
        Serial.print(axis + 1);
        Serial.print(" RMS current set to: ");
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
                tmc.microsteps(
                    static_cast<uint16_t>(value)
                );

                Serial.print("Axis ");
                Serial.print(axis + 1);
                Serial.print(" microsteps set to: ");
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
            tmc.en_spreadCycle(false);
            tmc.pwm_autoscale(true);

            Serial.print("Axis ");
            Serial.print(axis + 1);
            Serial.println(" mode set to StealthChop");
            return;
        }

        if (mode == "spread") {
            tmc.en_spreadCycle(true);

            Serial.print("Axis ");
            Serial.print(axis + 1);
            Serial.println(" mode set to SpreadCycle");
            return;
        }

        Serial.println(
            "Uzycie: mode stealth lub mode spread"
        );
        return;
    }

    // ------------------------------------------------
    // sine
    // ------------------------------------------------

    if (command == "sine") {
        if (motionExecutor == nullptr || trajectoryGenerator == nullptr) {
            Serial.println(
                "Komenda sine jest niezainicjalizowana."
            );
            return;
        }

        // Split arguments into tokens
        std::vector<String> tokens;
        String remaining = argument;
        remaining.trim();

        while (remaining.length() > 0) {
            int sep = remaining.indexOf(' ');

            if (sep < 0) {
                tokens.push_back(remaining);
                break;
            }

            tokens.push_back(remaining.substring(0, sep));
            remaining = remaining.substring(sep + 1);
            remaining.trim();
        }

        // Accept either 3 params (amplitude frequency dt) or 4 (amplitude frequency duration dt)
        if (tokens.size() != 3 && tokens.size() != 4) {
            Serial.println(
                "Uzycie: sine <amplitude> <frequency> [duration] <dt>"
            );
            return;
        }

        double amplitude = 0.0;
        double frequency = 0.0;
        double duration = 0.0;
        double timeStep = 0.0;

        if (!parseDoubleArgument(tokens[0], amplitude) ||
            !parseDoubleArgument(tokens[1], frequency)) {
            Serial.println(
                "Uzycie: sine <amplitude> <frequency> [duration] <dt>"
            );
            return;
        }

        if (tokens.size() == 3) {
            // amplitude frequency dt
            if (!parseDoubleArgument(tokens[2], timeStep)) {
                Serial.println(
                    "Uzycie: sine <amplitude> <frequency> [duration] <dt>"
                );
                return;
            }

            duration = 1.0 / frequency;
        } else {
            // amplitude frequency duration dt
            if (!parseDoubleArgument(tokens[2], duration) ||
                !parseDoubleArgument(tokens[3], timeStep)) {
                Serial.println(
                    "Uzycie: sine <amplitude> <frequency> [duration] <dt>"
                );
                return;
            }
        }

        if (amplitude < 0.0) {
            Serial.println("Amplituda nie moze byc ujemna");
            return;
        }

        if (frequency <= 0.0) {
            Serial.println("Czestotliwosc musi byc > 0");
            return;
        }

        if (duration <= 0.0) {
            Serial.println("Dlugosc trwania musi byc > 0");
            return;
        }

        if (timeStep <= 0.0) {
            Serial.println("Krok czasowy musi byc > 0");
            return;
        }

        std::vector<int> stepTrajectory;

        trajectoryGenerator->sinusoidalTrajectory(
            amplitude,
            frequency,
            duration,
            timeStep
        );

        // Stream positions directly into MotionExecutor to avoid heavy allocation/copy spikes.
        motionExecutor->startFromGenerator(trajectoryGenerator, timeStep, trajectoryGenerator->getStepsPerRevolution());

        if (!motor.isEnabled()) {
            motor.enable();
        }

        Serial.print("Axis ");
        Serial.print(axis + 1);
        Serial.print(" sine configured: amplitude=");
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
        if (firstToken == "1" || firstToken == "2") {
            printStatusForAxis(axis);
        } else {
            printStatus();
        }
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
