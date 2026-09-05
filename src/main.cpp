#include <Arduino.h>
#include <TMCStepper.h>

#include "MotorDriver.hpp"
#include "SerialCommandHandler.h"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"

// --------------------------------------------------
// Konfiguracja TMC2209
// --------------------------------------------------

constexpr float R_SENSE = 0.11f;
constexpr uint8_t TMC1_ADDRESS = 0b00;
constexpr uint8_t TMC2_ADDRESS = 0b00;

constexpr uint32_t DEBUG_BAUD_RATE = 115200;
constexpr uint32_t TMC1_BAUD_RATE = 115200;
constexpr uint32_t TMC2_BAUD_RATE = 115200;

// UART ESP32 -> TMC2209
constexpr int TMC1_RX_PIN = 16;
constexpr int TMC1_TX_PIN = 17;

constexpr int TMC2_RX_PIN = 21;
constexpr int TMC2_TX_PIN = 22;

// STEP / DIR / ENABLE
constexpr int STEP1_PIN = 25;
constexpr int DIR1_PIN = 26;
constexpr int ENABLE1_PIN = 18;

constexpr int STEP2_PIN = 32;
constexpr int DIR2_PIN = 33;
constexpr int ENABLE2_PIN = 27;

// Początkowe ustawienia
constexpr uint16_t INITIAL_RMS_CURRENT = 600;
constexpr uint16_t INITIAL_MICROSTEPS = 16;
constexpr uint32_t INITIAL_SPEED = 0;

// --------------------------------------------------
// Obiekty
// --------------------------------------------------

MotorDriver motor1(
    STEP1_PIN,
    DIR1_PIN,
    ENABLE1_PIN
);

MotorDriver motor2(
    STEP2_PIN,
    DIR2_PIN,
    ENABLE2_PIN
);

TMC2209Stepper tmc1(
    &Serial1,
    R_SENSE,
    TMC1_ADDRESS
);

TMC2209Stepper tmc2(
    &Serial2,
    R_SENSE,
    TMC2_ADDRESS
);

// --------------------------------------------------
// Obiekty do trajektorii i ruchu
// --------------------------------------------------

MotionExecutor motion_executor1(
    motor1
);

MotionExecutor motion_executor2(
    motor2
);

TrajectoryGenerator trajectory_generator1(16 * 16 * 100);
TrajectoryGenerator trajectory_generator2(16 * 16 * 100);

// --------------------------------------------------
// Obiekt obsługi komend szeregowych
// --------------------------------------------------

SerialCommandHandler serialCommandHandler(
    motor1,
    motor2,
    tmc1,
    tmc2,
    &motion_executor1,
    &motion_executor2,
    &trajectory_generator1,
    &trajectory_generator2
);

// --------------------------------------------------
// setup
// --------------------------------------------------

void setup() {
    Serial.begin(DEBUG_BAUD_RATE);

    delay(200);

    Serial.println();
    Serial.println("Start programu");

    motor1.begin();
Serial1.begin(
        TMC1_BAUD_RATE,
        SERIAL_8N1,
        TMC1_RX_PIN,
        TMC1_TX_PIN
    );

    motor2.begin();
    Serial2.begin(
        TMC2_BAUD_RATE,
        SERIAL_8N1,
        TMC2_RX_PIN,
        TMC2_TX_PIN
    );

    delay(100);

    // Podstawowa konfiguracja TMC2209.
    tmc1.begin();
    tmc2.begin();

    // Pin PDN_UART ma pracowac jako UART.
    tmc1.pdn_disable(true);
    tmc2.pdn_disable(true);

    // Ustawianie pradu przez rejestry UART.
    tmc1.I_scale_analog(false);
    tmc2.I_scale_analog(false);

    // Aktywacja stopnia mocy/choppera.
    tmc1.toff(4);
    tmc2.toff(4);

    // Prad silnika w mA RMS.
    tmc1.rms_current(INITIAL_RMS_CURRENT);
    tmc2.rms_current(INITIAL_RMS_CURRENT);

    // Liczba mikrokrokow.
    tmc1.microsteps(INITIAL_MICROSTEPS);
    tmc2.microsteps(INITIAL_MICROSTEPS);

    // false = StealthChop
    tmc1.en_spreadCycle(false);
    tmc2.en_spreadCycle(false);

    // Automatyczne dopasowanie PWM StealthChop.
    tmc1.pwm_autoscale(true);
    tmc2.pwm_autoscale(true);

    motor1.setDirection(true);
    motor1.setSpeed(INITIAL_SPEED);
    motor1.enable();
    motor1.start();
    
    motor2.setDirection(true);
    motor2.setSpeed(INITIAL_SPEED);
    motor2.enable();
    motor2.start();

    Serial.println("Drivers configured");

    serialCommandHandler.printHelp();
    serialCommandHandler.printStatus();
}

// --------------------------------------------------
// loop
// --------------------------------------------------

void loop() {
    // Ta metoda musi wykonywac sie bardzo czesto.
    motor1.run();
    motor2.run();

    if (motion_executor1.hasActiveTrajectory()) {
        motion_executor1.update();
    }

    if (motion_executor2.hasActiveTrajectory()) {
        motion_executor2.update();
    }

    // Odczyt jest nieblokujacy.
    serialCommandHandler.readSerialCommands();


}


//sine <amplitude> <frequency> [duration] <dt>
//sine 360 0.1 60 0.1

