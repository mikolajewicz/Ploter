#include <Arduino.h>
#include <TMCStepper.h>

#include "MotorDriver.hpp"
#include "SerialCommandHandler.h"

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
// Obiekt obsługi komend szeregowych
// --------------------------------------------------

SerialCommandHandler serialCommandHandler(
    motor,
    tmc
);

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

    serialCommandHandler.printHelp();
    serialCommandHandler.printStatus();
}

// --------------------------------------------------
// loop
// --------------------------------------------------

void loop() {
    // Ta metoda musi wykonywac sie bardzo czesto.
    motor.run();

    // Odczyt jest nieblokujacy.
    serialCommandHandler.readSerialCommands();
}