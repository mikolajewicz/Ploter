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
#include <WiFi.h>
#include "secrets.h"
#include <ArduinoOTA.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// void testGCode();

// --------------------------------------------------
// Konfiguracja WIFI
// --------------------------------------------------

void setupWiFi()
{
    WiFi.mode(WIFI_STA);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to WiFi");

    unsigned long start = millis();

    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - start < 10000
    )
    {
        delay(100);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println();
        Serial.print("ESP IP: ");
        Serial.println(WiFi.localIP());

        if (WiFi.status() == WL_CONNECTED)
        {
            ArduinoOTA.setHostname("ploter-esp32");
            ArduinoOTA.setPassword(OTA_PASSWORD);

            ArduinoOTA.onStart([]() {
                // Nie rozpoczynaj aktualizacji podczas ruchu.
                // Docelowo tutaj wyłączamy sterowanie silnikami
                // i anulujemy wszystkie ruchy.
                Serial.println("OTA START");
            });

            ArduinoOTA.begin();

            Serial.println("OTA READY");
            Serial.println(WiFi.localIP());
        }
    }
    else
    {
        Serial.println("WiFi connection failed");
    }

}

void maintainWiFi()
{
    static uint32_t lastReconnectAttempt = 0;

    if (WiFi.status() == WL_CONNECTED)
        return;

    if (millis() - lastReconnectAttempt < 5000)
        return;

    lastReconnectAttempt = millis();

    Serial.println("WiFi reconnecting...");
    WiFi.reconnect();
}

void otaTask(void* parameter)
{
    while (true)
    {
        maintainWiFi();

        if (WiFi.status() == WL_CONNECTED)
        {
            ArduinoOTA.handle();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --------------------------------------------------
// Konfiguracja TMC2209
// --------------------------------------------------



constexpr double HOME_ANGLE_M1 = 141.866 - 180.0;
constexpr double HOME_ANGLE_M2 = 180.0 - 141.866;

constexpr double HOME_OFFSET_M1 = 101.0 + HOME_ANGLE_M1;
constexpr double HOME_OFFSET_M2 = -101.0 + HOME_ANGLE_M2;

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

void testMotor1Direction();

Kinematics Solver;

// --------------------------------------------------
// Obiekt obsługi komend szeregowych
// --------------------------------------------------

PenController pen(
    SERVO_PIN,
    60,     // góra
    90      // dół
);

bool requestHomeSequence()
{
    // Bazowanie jest już w toku
    if (homeState >= 1 && homeState <= 3) {
        return false;
    }

    // Podnieś pióro przed bazowaniem
    pen.up();

    startHomeSequence();

    return true;
}

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

bool isHomeSequenceActive()
{
    return homeState >= 1 && homeState <= 3;
}


// --------------------------------------------------
// Serwer TCP do odbioru komend
// --------------------------------------------------


constexpr uint16_t TCP_PORT = 5000;

struct TcpCommand
{
    char text[112];
};

WiFiServer tcpServer(TCP_PORT);

QueueHandle_t tcpCommands = nullptr;
QueueHandle_t tabletCommands = nullptr;

struct TcpResponse
{
    char text[128];
};

QueueHandle_t tcpResponses = nullptr;
QueueHandle_t tcpPosition = nullptr;

void sendResponse(const char* message)
{
    Serial.println(message);

    TcpResponse response = {};
    snprintf(response.text, sizeof(response.text), "%s\n", message);

    if (strncmp(message, "POS ", 4) == 0)
    {
        if (tcpPosition != nullptr)
            xQueueOverwrite(tcpPosition, &response);
    }
    else
    {
        if (tcpResponses != nullptr &&
            xQueueSend(tcpResponses, &response, 0) != pdTRUE)
        {
            Serial.println("WARN TCP response queue full");
        }
    }
}


void tcpTask(void* parameter)
{
    bool serverStarted = false;

    WiFiClient client;

    char buffer[112] = {};
    size_t length = 0;
    bool discardLine = false;

    TcpResponse pendingResponse = {};
    size_t pendingOffset = 0;
    bool responsePending = false;

    auto resetConnection = [&]()
    {
        client.stop();

        length = 0;
        discardLine = false;

        responsePending = false;
        pendingOffset = 0;

        // Usuwamy odpowiedzi ze starego połączenia.
        if (tcpResponses != nullptr)
            xQueueReset(tcpResponses);

        if (tcpPosition != nullptr)
            xQueueReset(tcpPosition);

        // Usuwamy stare pozycje tabletu.
        if (tabletCommands != nullptr)
            xQueueReset(tabletCommands);

        // Uwaga: tcpCommands nie jest tu czyszczone,
        // bo może zawierać polecenia sterujące.
    };

    while (true)
    {
        // -----------------------------------------
        // Brak Wi-Fi
        // -----------------------------------------

        if (WiFi.status() != WL_CONNECTED)
        {
            resetConnection();

            if (serverStarted)
            {
                tcpServer.end();
                serverStarted = false;

                Serial.println("TCP server stopped");
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // -----------------------------------------
        // Wi-Fi zostało połączone
        // -----------------------------------------

        if (!serverStarted)
        {
            tcpServer.begin();
            serverStarted = true;

            Serial.println("TCP server ready on port 5000");
        }

        // -----------------------------------------
        // Akceptowanie klienta
        // -----------------------------------------

        if (!client || !client.connected())
        {
            resetConnection();

            client = tcpServer.available();

            if (client && client.connected())
            {
                client.setNoDelay(true);
                client.setTimeout(20);

                client.println("TCP CONNECTED");

                Serial.println("TCP client connected");

                // testGCode();
            }

            
        }

        // -----------------------------------------
        // Odbieranie komend
        // -----------------------------------------

        if (client && client.connected())
        {
            int budget = 128;

            while (client.available() > 0 && budget-- > 0)
            {
                int value = client.read();

                if (value < 0)
                    break;

                char c = static_cast<char>(value);

                if (c == '\r')
                    continue;

                if (c == '\n')
                {
                    if (!discardLine && length > 0)
                    {
                        buffer[length] = '\0';

                        TcpCommand command = {};
                        memcpy(
                            command.text,
                            buffer,
                            length + 1
                        );

                        if (strncmp(command.text, "tablet ", 7) == 0)
                        {
                            // Zachowujemy tylko najnowszy cel.
                            if (tabletCommands != nullptr)
                                xQueueOverwrite(
                                    tabletCommands,
                                    &command
                                );
                        }
                        else if (strcmp(command.text, "stop") == 0)
                        {
                            // STOP ma pierwszeństwo.
                            if (tcpCommands != nullptr)
                            {
                                if (xQueueSendToFront(
                                        tcpCommands,
                                        &command,
                                        0
                                    ) != pdTRUE)
                                {
                                    Serial.println(
                                        "WARN TCP command queue full"
                                    );
                                }
                            }
                        }
                        else
                        {
                            if (tcpCommands != nullptr)
                            {
                                if (xQueueSend(
                                        tcpCommands,
                                        &command,
                                        0
                                    ) != pdTRUE)
                                {
                                    Serial.println(
                                        "WARN TCP command queue full"
                                    );
                                }
                            }
                        }
                    }

                    length = 0;
                    discardLine = false;
                }
                else if (!discardLine)
                {
                    if (length < sizeof(buffer) - 1)
                    {
                        buffer[length++] = c;
                    }
                    else
                    {
                        // Odrzuć całą zbyt długą linię.
                        length = 0;
                        discardLine = true;
                    }
                }
            }
        }

        // -----------------------------------------
        // Wysyłanie odpowiedzi do Pythona
        // -----------------------------------------

        if (client && client.connected())
        {
            if (!responsePending)
            {
                // Najpierw ważne komunikaty.
                if (
                    tcpResponses != nullptr &&
                    xQueueReceive(
                        tcpResponses,
                        &pendingResponse,
                        0
                    ) == pdTRUE
                )
                {
                    responsePending = true;
                    pendingOffset = 0;
                }
                // Następnie najnowsza pozycja.
                else if (
                    tcpPosition != nullptr &&
                    xQueueReceive(
                        tcpPosition,
                        &pendingResponse,
                        0
                    ) == pdTRUE
                )
                {
                    responsePending = true;
                    pendingOffset = 0;
                }
            }

            if (responsePending)
            {
                size_t total = strlen(pendingResponse.text);

                if (pendingOffset < total)
                {
                    size_t remaining = total - pendingOffset;

                    // Na Core 0; krótki timeout.
                    size_t written = client.write(
                        reinterpret_cast<const uint8_t*>(
                            pendingResponse.text + pendingOffset
                        ),
                        remaining
                    );

                    pendingOffset += written;
                }

                if (pendingOffset >= total)
                {
                    responsePending = false;
                    pendingOffset = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void processTcpCommands()
{
    TcpCommand command;

    // Najpierw komenda sterująca, np. HOME lub STOP.
    if (tcpCommands != nullptr &&
        xQueueReceive(tcpCommands, &command, 0) == pdTRUE)
    {
        serialCommandHandler.processCommand(command.text);
    }

    // Następnie tylko najnowsza pozycja tabletu.
    if (tabletCommands != nullptr &&
        xQueueReceive(tabletCommands, &command, 0) == pdTRUE)
    {
        serialCommandHandler.processCommand(command.text);
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

   

    setupWiFi();

    
    tcpCommands = xQueueCreate(16, sizeof(TcpCommand));
    tabletCommands = xQueueCreate(1, sizeof(TcpCommand));

    tcpResponses = xQueueCreate(32, sizeof(TcpResponse));
    tcpPosition = xQueueCreate(1, sizeof(TcpResponse));

    if (
        tcpCommands == nullptr ||
        tabletCommands == nullptr ||
        tcpResponses == nullptr ||
        tcpPosition == nullptr
    )
    {
        Serial.println("TCP queue allocation failed");
    }

    else 
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
            Serial.println("TCP task creation failed");
        }
    }


    BaseType_t result = xTaskCreatePinnedToCore(
        otaTask,
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


    startHomeSequence();
}

// --------------------------------------------------
// loop
// --------------------------------------------------


void loop()
{
    serialCommandHandler.readSerialCommands();
    processTcpCommands();


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

    if (
        homeState == 4 &&
        !motion_executor1.isActive() &&
        !motion_executor2.isActive()
    )
    {
        if (motionManager.isTrajectoryReady())
        {
            motionManager.startPreparedMotion();
        }
        else
        {
            motionManager.followTarget();
        }
    }


    static uint32_t lastPositionSend = 0;

    if (
        homeState == 4 &&
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
                homeState = 4;

                motor1.setStepCount(
                    angleToSteps(HOME_ANGLE_M1)
                );

                motor2.setStepCount(
                    angleToSteps(HOME_ANGLE_M2)
                );

                tmc1.rms_current(INITIAL_RMS_CURRENT);
                tmc2.rms_current(INITIAL_RMS_CURRENT);

                sendResponse("HOME sequence finished");
            }

            

            break;
        }

        case 4:
        {
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
    motionManager.clearLiveTarget();

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

    sendResponse("HOME sequence started");  
}

int64_t angleToSteps(double angle)
{
    return llround(
        angle / 360.0 *
        stepsPerRevolution
    );
}

void testMotor1Direction()
{
    Serial.println("=== M1 TEST ===");

    motor1.enable();

    Serial.print("Start stepCount: ");
    Serial.println(motor1.getStepCount());

    // DIR = false
    motor1.setDirection(false);

    Serial.println("M1 DIR=false - 200 steps");

    for (int i = 0; i < 200; i++)
    {
        motor1.step();
        delayMicroseconds(1000);
    }

    Serial.print("stepCount: ");
    Serial.println(motor1.getStepCount());

    delay(2000);

    // DIR = true
    motor1.setDirection(true);

    Serial.println("M1 DIR=true - 200 steps");

    for (int i = 0; i < 200; i++)
    {
        motor1.step();
        delayMicroseconds(1000);
    }

    Serial.print("stepCount: ");
    Serial.println(motor1.getStepCount());

    Serial.println("=== END ===");
}









