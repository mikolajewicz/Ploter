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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "GCodeStorage.hpp"
#include "GCodeProgram.hpp"
#include "esp_system.h"
#include "CommunicationManager.hpp"
#include "NetworkManager.hpp"
#include "HomingSequence.hpp"
#include "PlotterController.hpp"

RTC_DATA_ATTR uint32_t lastCrashStage = 0;

esp_reset_reason_t bootResetReason =
    ESP_RST_UNKNOWN;

constexpr bool BYPASS_HOME = false;

// --------------------------------------------------
// Konfiguracja TMC2209
// --------------------------------------------------

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
constexpr uint16_t INITIAL_RMS_CURRENT = 900;
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

void testMotor1Direction();
void testReadGCode();

Kinematics Solver;

// --------------------------------------------------
// Obiekt obsługi komend szeregowych
// --------------------------------------------------

PenController pen(
    SERVO_PIN,
    135,     // góra
    90      // dół
);

GCodeStorage gcodeStorage;

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

HomingSequence homingSequence(
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

    motionManager,
    pen,

    stepsPerRevolution,
    INITIAL_RMS_CURRENT,
    TIME_STEP,
    BYPASS_HOME
);

GCodeProgram gcodeProgram(
    gcodeStorage,
    motionManager
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

PlotterController plotterController(
    motor1,
    motor2,
    motionManager,
    homingSequence,
    pen,
    gcodeProgram,
    serialCommandHandler
);

// --------------------------------------------------
// setup
// --------------------------------------------------

void setup() {
    
    Serial.begin(DEBUG_BAUD_RATE);
    delay(200);

    bootResetReason = esp_reset_reason();

    Serial.println();
    Serial.print("RESET REASON: ");
    Serial.println(
        static_cast<int>(esp_reset_reason())
    );

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

    if (!gcodeStorage.begin())
    {
        Serial.println(
            "GCode storage ERROR"
        );
    }
    else
    {
        Serial.println(
            "GCode storage ready"
        );
    }


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

    networkManager.begin();

    if (communication.begin())
    {
        BaseType_t result = xTaskCreatePinnedToCore(
            tcpTask,
            "TCP",
            6144,
            nullptr,
            1,
            nullptr,
            0
        );

        if (result != pdPASS)
        {
            Serial.println(
                "TCP task creation failed"
            );
        }
    }
    else
    {
        Serial.println(
            "Communication initialization failed"
        );
    }



    BaseType_t result = xTaskCreatePinnedToCore(
        NetworkManager::otaTask,
        "OTA",
        6144,
        nullptr,
        1,
        nullptr,
        0
    );

    if (result != pdPASS)
    {
        Serial.println("OTA task ERROR");
    }

    homingSequence.start();
}

// --------------------------------------------------
// loop
// --------------------------------------------------



void loop()
{

    // ----------------------------------------
    // Komunikacja USB i TCP
    // ----------------------------------------

    communication.readUsbCommands();

    plotterController.processCommands();

    serialCommandHandler.updatePendingMotion();

    // ----------------------------------------
    // Homing
    // ----------------------------------------

    homingSequence.update();

    // ----------------------------------------
    // Generowanie STEP
    // ----------------------------------------

    motor1.run();
    motor2.run();

    // ----------------------------------------
    // Trajektorie
    // ----------------------------------------

    motion_executor1.update();
    motion_executor2.update();

    motionManager.updatePause();

    // ----------------------------------------
    // G-code
    // ----------------------------------------

    gcodeProgram.update();

    // ----------------------------------------
    // Wykonywanie przygotowanych ruchów
    // ----------------------------------------

    if (
        homingSequence.isReady() &&
        !motionManager.isPauseActive() &&
        !motion_executor1.isActive() &&
        !motion_executor2.isActive()
    )
    {
        if (motionManager.isTrajectoryReady())
        {
            motionManager.startPreparedMotion();
        }
        else if (!gcodeProgram.isRunning())
        {
            motionManager.followTarget();
        }
    }

    // ----------------------------------------
    // Wysyłanie pozycji plotera
    // ----------------------------------------

    static uint32_t lastPositionSend = 0;

    if (
        homingSequence.isReady() &&
        millis() - lastPositionSend >= 20
    )
    {
        lastPositionSend = millis();

        if (motionManager.calculateCurrentPosition())
        {
            char message[80];

            snprintf(
                message,
                sizeof(message),
                "POS %.3f %.3f",
                motionManager.getCurrentX(),
                motionManager.getCurrentY()
            );

            sendResponse(message);
        }
    }

}

void testReadGCode()
{
    if (!gcodeStorage.openProgram())
    {
        sendResponse(
            "GCODE READ ERROR"
        );

        return;
    }

    sendResponse(
        "GCODE FILE BEGIN"
    );

    String line;
    int lineNumber = 1;

    while (
        gcodeStorage.readLine(line)
    )
    {
        char response[160];

        snprintf(
            response,
            sizeof(response),
            "GCODE LINE %d: %s",
            lineNumber,
            line.c_str()
        );

        sendResponse(
            response
        );

        lineNumber++;
    }

    gcodeStorage.closeProgram();

    sendResponse(
        "GCODE FILE END"
    );
}






