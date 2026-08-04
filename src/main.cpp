#include <Arduino.h>
#include <TMCStepper.h>

#include "MotorDriver.hpp"

// --------------------------------------------------
// Konfiguracja TMC2209
// --------------------------------------------------

constexpr float R_SENSE = 0.11f;
constexpr uint8_t TMC_ADDRESS = 0b00;

constexpr uint32_t DEBUG_BAUD_RATE = 115200;
constexpr uint32_t TMC_BAUD_RATE = 115200;

// UART ESP32 -> TMC2209
constexpr int TMC_RX_PIN = 16;
constexpr int TMC_TX_PIN = 17;

// STEP / DIR / ENABLE
constexpr int STEP_PIN = 32;
constexpr int DIR_PIN = 33;
constexpr int ENABLE_PIN = 27;

// Początkowe ustawienia
constexpr uint16_t INITIAL_RMS_CURRENT = 600;
constexpr uint16_t INITIAL_MICROSTEPS = 16;
constexpr uint32_t INITIAL_SPEED = 0;

// --------------------------------------------------
// Obiekty
// --------------------------------------------------

MotorDriver motor(
    STEP_PIN,
    DIR_PIN,
    ENABLE_PIN
);

TMC2209Stepper tmc(
    &Serial2,
    R_SENSE,
    TMC_ADDRESS
);

// --------------------------------------------------
// Bufor komend
// --------------------------------------------------

String serialCommand;

// --------------------------------------------------
// Funkcje pomocnicze
// --------------------------------------------------

void printHelp() {
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

void printStatus() {
    Serial.println();
    Serial.println("----- STATUS -----");

    Serial.print("Enabled: ");
    Serial.println(
        motor.isEnabled() ? "yes" : "no"
    );

    Serial.print("Running: ");
    Serial.println(
        motor.isRunning() ? "yes" : "no"
    );

    Serial.print("Speed: ");
    Serial.print(motor.getSpeed());
    Serial.println(" steps/s");

    Serial.print("Direction: ");
    Serial.println(
        motor.getDirection() ? "1" : "0"
    );

    Serial.print("IFCNT: ");
    Serial.println(tmc.IFCNT());

    Serial.print("TMC version: ");
    Serial.println(tmc.version());

    Serial.print("DRV_STATUS: 0x");
    Serial.println(tmc.DRV_STATUS(), HEX);

    Serial.println("------------------");
    Serial.println();
}

bool parseLongArgument(
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

void handleSerialCommand(String line) {
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
        motor.enable();

        Serial.println("Motor enabled");
        return;
    }

    // ------------------------------------------------
    // disable
    // ------------------------------------------------

    if (command == "disable") {
        motor.disable();

        Serial.println("Motor disabled");
        return;
    }

    // ------------------------------------------------
    // start
    // ------------------------------------------------

    if (command == "start") {
        if (!motor.isEnabled()) {
            motor.enable();
        }

        motor.start();

        if (motor.getSpeed() == 0) {
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
        motor.stop();

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

        motor.setSpeed(
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
            motor.setDirection(false);

            Serial.println("Direction set to: 0");
            return;
        }

        if (argument == "1") {
            motor.setDirection(true);

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

        tmc.rms_current(
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
                tmc.microsteps(
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
            tmc.en_spreadCycle(false);
            tmc.pwm_autoscale(true);

            Serial.println(
                "Mode set to StealthChop"
            );
            return;
        }

        if (mode == "spread") {
            tmc.en_spreadCycle(true);

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

void readSerialCommands() {
    while (Serial.available() > 0) {
        char character = Serial.read();

        if (character == '\r') {
            continue;
        }

        if (character == '\n') {
            handleSerialCommand(serialCommand);
            serialCommand = "";
            continue;
        }

        // Ochrona przed nieograniczonym wzrostem String.
        if (serialCommand.length() < 100) {
            serialCommand += character;
        } else {
            serialCommand = "";

            Serial.println(
                "Komenda jest zbyt dluga"
            );
        }
    }
}

// --------------------------------------------------
// setup
// --------------------------------------------------

void setup() {
    Serial.begin(DEBUG_BAUD_RATE);

    delay(200);

    Serial.println();
    Serial.println("Start programu");

    motor.begin();

    Serial2.begin(
        TMC_BAUD_RATE,
        SERIAL_8N1,
        TMC_RX_PIN,
        TMC_TX_PIN
    );

    delay(100);

    // Podstawowa konfiguracja TMC2209.
    tmc.begin();

    // Pin PDN_UART ma pracowac jako UART.
    tmc.pdn_disable(true);

    // Ustawianie pradu przez rejestry UART.
    tmc.I_scale_analog(false);

    // Aktywacja stopnia mocy/choppera.
    tmc.toff(4);

    // Prad silnika w mA RMS.
    tmc.rms_current(INITIAL_RMS_CURRENT);

    // Liczba mikrokrokow.
    tmc.microsteps(INITIAL_MICROSTEPS);

    // false = StealthChop
    tmc.en_spreadCycle(false);

    // Automatyczne dopasowanie PWM StealthChop.
    tmc.pwm_autoscale(true);

    motor.setDirection(true);
    motor.setSpeed(INITIAL_SPEED);
    motor.enable();
    motor.start();

    Serial.println("TMC2209 configured");

    printHelp();
    printStatus();
}

// --------------------------------------------------
// loop
// --------------------------------------------------

void loop() {
    // Ta metoda musi wykonywac sie bardzo czesto.
    motor.run();

    // Odczyt jest nieblokujacy.
    readSerialCommands();
}