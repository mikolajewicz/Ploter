#include <Arduino.h>
#include <TMCStepper.h>

#include "MotorDriver.hpp"
#include "SerialCommandHandler.h"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"
#include "Homing.hpp"
#include "Kinematics.hpp"
#include "MotionManager.hpp"
#include "PenController.hpp"

// --------------------------------------------------
// Konfiguracja TMC2209
// --------------------------------------------------

constexpr double HOME_OFFSET_M1 = 99.0;
constexpr double HOME_OFFSET_M2 = -53.0;

constexpr double HOME_ANGLE_M1 = 141.9;
constexpr double HOME_ANGLE_M2 = 38.1;

constexpr double HOME_OFFSET_TIME = 2.0;
constexpr double HOME_OFFSET_ACCEL_TIME = 0.5;

constexpr int SERVO_PIN = 23;

constexpr double TIME_STEP = 0.01;
constexpr double LINE_SPEED = 100.0;

constexpr unsigned int stepsPerRevolution = 4 * 16 * 400;

constexpr float R_SENSE = 0.11f;
constexpr uint8_t TMC1_ADDRESS = 0b00;
constexpr uint8_t TMC2_ADDRESS = 0b00;

constexpr uint32_t DEBUG_BAUD_RATE = 115200;
constexpr uint32_t TMC1_BAUD_RATE = 115200;
constexpr uint32_t TMC2_BAUD_RATE = 115200;

// UART ESP32 -> TMC2209

// TMC1:
// GPIO21 = RX, GPIO22 = TX przez R1 1k
constexpr int TMC1_RX_PIN = 21;
constexpr int TMC1_TX_PIN = 22;

// TMC2:
// GPIO16 = RX, GPIO17 = TX przez R2 1k
constexpr int TMC2_RX_PIN = 16;
constexpr int TMC2_TX_PIN = 17;


// STEP / DIR / ENABLE

// Motor 1
constexpr int STEP1_PIN = 32;
constexpr int DIR1_PIN = 33;
constexpr int ENABLE1_PIN = 27;

// Motor 2
constexpr int STEP2_PIN = 25;
constexpr int DIR2_PIN = 26;
constexpr int ENABLE2_PIN = 18;


// DIAG

constexpr int DIAG1_PIN = 34;
constexpr int DIAG2_PIN = 35;

// Początkowe ustawienia
constexpr uint16_t INITIAL_RMS_CURRENT = 600;
constexpr uint16_t INITIAL_MICROSTEPS = 16;
constexpr uint32_t INITIAL_SPEED = 0;

int sequenceState = 0;

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
    motor1,
    0.01
);

MotionExecutor motion_executor2(
    motor2,
    0.01
);

TrajectoryGenerator trajectory_generator1(stepsPerRevolution);
TrajectoryGenerator trajectory_generator2(stepsPerRevolution);


// --------------------------------------------------
// HOMING
// --------------------------------------------------

bool motor1Homed = false;
bool motor2Homed = false;

Homing homingMotor1(
    motor1,
    tmc1,
    true,       // kierunek homingu m1
    DIAG1_PIN
);

Homing homingMotor2(
    motor2,
    tmc2,
    false,      // kierunek homingu m2
    DIAG2_PIN
);

int homeState = 0;

void startHomeSequence();

void updateHomeSequence(
    bool homing1Finished,
    bool homing2Finished
);

int64_t angleToSteps(double angle);

Kinematics Solver;

// --------------------------------------------------
// Obiekt obsługi komend szeregowych
// --------------------------------------------------

PenController pen(
    SERVO_PIN,
    60,     // góra
    90      // dół
);

MotionManager motionManager(
    motor1,
    tmc1,
    motion_executor1,
    trajectory_generator1,
    homingMotor1,

    motor2,
    tmc2,
    motion_executor2,
    trajectory_generator2,
    homingMotor2,

    Solver,
    pen
);

SerialCommandHandler serialCommandHandler(
    motor1,
    tmc1,
    motion_executor1,
    trajectory_generator1,
    homingMotor1,

    motor2,
    tmc2,
    motion_executor2,
    trajectory_generator2,
    homingMotor2,

    motionManager
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

    pinMode(DIAG1_PIN, INPUT);
    pinMode(DIAG2_PIN, INPUT);

    if (!motionManager.beginPlanner()) {
    Serial.println("Planner ERROR");
    }

    pen.begin();

    startHomeSequence();
}

// --------------------------------------------------
// loop
// --------------------------------------------------


void loop()
{
    serialCommandHandler.readSerialCommands();

    // Aktualizacja homingu
    bool homing1Finished =
        homingMotor1.update();

    bool homing2Finished =
        homingMotor2.update();

    // Aktualizacja sekwencji HOME
    updateHomeSequence(
        homing1Finished,
        homing2Finished
    );

    // Generowanie STEP
    motor1.run();
    motor2.run();

    // Trajektorie
    motion_executor1.update();
    motion_executor2.update();
}

void updateHomeSequence(
    bool homing1Finished,
    bool homing2Finished
)
{
    switch (homeState)
    {
        case 0:
        {
            break;
        }

        case 1:
        {
            if (homing1Finished)
            {
                motor1Homed = true;
                Serial.println("M1 homed");
            }

            if (homing2Finished)
            {
                motor2Homed = true;
                Serial.println("M2 homed");
            }

            if (motor1Homed && motor2Homed)
            {
                Serial.println("Both motors homed");

                homeState = 2;
            }

            break;
        }

        case 2:
        {
            bool ok1 =
                trajectory_generator1.trapezoidalProfile(
                    HOME_OFFSET_M1,
                    HOME_OFFSET_TIME,
                    HOME_OFFSET_ACCEL_TIME,
                    TIME_STEP
                );

            bool ok2 =
                trajectory_generator2.trapezoidalProfile(
                    HOME_OFFSET_M2,
                    HOME_OFFSET_TIME,
                    HOME_OFFSET_ACCEL_TIME,
                    TIME_STEP
                );

            if (!ok1 || !ok2)
            {
                Serial.println(
                    "HOME offset trajectory ERROR"
                );

                homeState = 99;
                break;
            }

            trajectory_generator1.convertToSteps();
            trajectory_generator2.convertToSteps();

            motion_executor1.setTimeStep(TIME_STEP);
            motion_executor2.setTimeStep(TIME_STEP);

            motion_executor1.start(
                trajectory_generator1.takeStepTrajectory(),
                TIME_STEP
            );

            motion_executor2.start(
                trajectory_generator2.takeStepTrajectory(),
                TIME_STEP
            );

            Serial.println(
                "Moving from home to zero"
            );

            homeState = 3;

            break;
        }

        case 3:
        {
            if (
                !motion_executor1.isActive() &&
                !motion_executor2.isActive()
            )
            {
                Serial.println(
                    "HOME sequence finished"
                );

                homeState = 4;

                motor1.setStepCount(
                    angleToSteps(HOME_ANGLE_M1)
                );

                motor2.setStepCount(
                    angleToSteps(HOME_ANGLE_M2)
                );
            }

            

            break;
        }

        case 4:
        {
            // gotowe
            break;
        }

        case 99:
        {
            motor1.stop();
            motor2.stop();

            motion_executor1.stop();
            motion_executor2.stop();

            break;
        }
    }
}

void startHomeSequence()
{
    motor1Homed = false;
    motor2Homed = false;

    motion_executor1.stop();
    motion_executor2.stop();

    motor1.stop();
    motor2.stop();

    motor1.enable();
    motor2.enable();

    homingMotor1.home();
    homingMotor2.home();

    homeState = 1;

    Serial.println("HOME sequence started");
}

int64_t angleToSteps(double angle)
{
    return llround(
        angle / 360.0 *
        stepsPerRevolution
    );
}











