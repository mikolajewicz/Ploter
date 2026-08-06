#include "SerialCommandHandler.h"

#include <TMCStepper.h>
#include "MotorDriver.hpp"

SerialCommandHandler::SerialCommandHandler(
    MotorDriver& motor,
    TMC2209Stepper& tmc
)
    : motor_(motor),
      tmc_(tmc) {
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
    Serial.println("  status");
    Serial.println("  help");
    Serial.println();

    Serial.println("Przyklady:");
    Serial.println("  speed 1000");
    Serial.println("  dir 0");
    Serial.println("  rms 700");
    Serial.println("  microsteps 16");
    Serial.println();
}

void SerialCommandHandler::printStatus() {
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

void SerialCommandHandler::handleSerialCommand(String line) {
    line.trim();

    if (line.length() == 0) {
        return;
    }

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
