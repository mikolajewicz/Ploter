#include "SerialCommandHandler.h"

#include <TMCStepper.h>
#include "MotorDriver.hpp"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"
#include "Homing.hpp"
#include "Kinematics.hpp"
#include "MotionManager.hpp"

SerialCommandHandler::SerialCommandHandler(
    MotorDriver& motor1,
    TMC2209Stepper& tmc1,
    MotionExecutor& motionExecutor1,
    TrajectoryGenerator& trajectoryGenerator1,
    Homing& homing1,

    MotorDriver& motor2,
    TMC2209Stepper& tmc2,
    MotionExecutor& motionExecutor2,
    TrajectoryGenerator& trajectoryGenerator2,
    Homing& homing2,

    MotionManager& motionManager
)
    : motors_{&motor1, &motor2},
      tmcs_{&tmc1, &tmc2},
      motionExecutors_{&motionExecutor1, &motionExecutor2},
      trajectoryGenerators_{&trajectoryGenerator1, &trajectoryGenerator2},
      homings_{&homing1, &homing2},
      motionManager(motionManager)

{
}

void SerialCommandHandler::printHelp()
{
    Serial.println();
    Serial.println("========== COMMANDS ==========");

    Serial.println();
    Serial.println("GLOBAL:");
    Serial.println("  help");
    Serial.println("  stop");

    Serial.println();
    Serial.println("MOTOR:");
    Serial.println("  m1 enable");
    Serial.println("  m1 disable");
    Serial.println("  m1 start");
    Serial.println("  m1 stop");
    Serial.println("  m1 speed <steps/s>");
    Serial.println("  m1 dir <0|1>");
    Serial.println("  m1 rms <mA>");
    Serial.println("  m1 microsteps <value>");
    Serial.println("  m1 mode <stealth|spread>");
    Serial.println("  m1 home");
    Serial.println("  m1 status");

    Serial.println();
    Serial.println("  m2 ...");

    Serial.println();
    Serial.println("MOTION:");

    Serial.println(
        "  motion a2b "
        "<Ax> <Ay> <Bx> <By> <time> <dt>"
    );

    Serial.println(
        "  motion line "
        "<Ax> <Ay> <Bx> <By> <speed> <dt>"
    );

    Serial.println();
    Serial.println("TABLET:");
    Serial.println(
        "  tablet <x> <y> <pressure> <inside>"
    );

    Serial.println();
    Serial.println("Examples:");
    Serial.println("  m1 speed 1000");
    Serial.println("  m1 start");
    Serial.println("  m2 rms 600");
    Serial.println("  m2 status");
    Serial.println("  tablet 0 0 1 1");
    Serial.println("  stop");

    Serial.println();
    Serial.println("==============================");
}

void SerialCommandHandler::printStatus(
    uint8_t motorNumber
)
{
    if (motorNumber < 1 || motorNumber > 2)
    {
        Serial.println(
            "ERR: motor must be 1 or 2"
        );
        return;
    }

    uint8_t motorIndex = motorNumber - 1;

    MotorDriver& motor =
        *motors_[motorIndex];

    TMC2209Stepper& tmc =
        *tmcs_[motorIndex];

    Serial.println();

    Serial.print("----- M");
    Serial.print(motorNumber);
    Serial.println(" STATUS -----");

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
    Serial.println(
        tmc.DRV_STATUS(),
        HEX
    );

    Serial.println("--------------------");
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

void SerialCommandHandler::handleSerialCommand(
    String line
)
{
    line.trim();

    if (line.length() == 0)
    {
        return;
    }

    // Pierwsze slowo komendy
    int separator = line.indexOf(' ');

    String group;
    String rest;

    if (separator < 0)
    {
        group = line;
        rest = "";
    }
    else
    {
        group = line.substring(
            0,
            separator
        );

        rest = line.substring(
            separator + 1
        );

        rest.trim();
    }

    group.toLowerCase();

    // --------------------------------------------
    // GLOBAL
    // --------------------------------------------

    if (group == "help")
    {
        printHelp();
        return;
    }

    if (group == "stop")
    {
        stopAll();

        Serial.println("ALL STOPPED");

        return;
    }

    // --------------------------------------------
    // TABLET
    // --------------------------------------------

    if (group == "tablet")
    {
        // Przekazujemy cala linie, bo parser
        // oczekuje:
        // tablet x y pressure inside

        handleTabletCommand(line);

        return;
    }

    // --------------------------------------------
    //  MOTION
    // --------------------------------------------


    if (group == "motion")
    {
        handleMotionCommand(rest);
        return;
    }


    // --------------------------------------------
    // MOTOR 1
    // --------------------------------------------

    if (group == "m1")
    {
        handleMotorCommand(
            0,
            rest
        );

        return;
    }

    // --------------------------------------------
    // MOTOR 2
    // --------------------------------------------

    if (group == "m2")
    {
        handleMotorCommand(
            1,
            rest
        );

        return;
    }

    

    // --------------------------------------------
    // UNKNOWN
    // --------------------------------------------

    Serial.print("ERR unknown command: ");
    Serial.println(group);
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

    updatePendingMotion();
}

void SerialCommandHandler::handleMotorCommand(
    uint8_t motorIndex,
    String line
)
{
    if (motorIndex > 1)
    {
        Serial.println("ERR invalid motor");
        return;
    }

    line.trim();

    if (line.length() == 0)
    {
        Serial.println(
            "ERR missing motor command"
        );
        return;
    }

    MotorDriver& motor =
        *motors_[motorIndex];

    TMC2209Stepper& tmc =
        *tmcs_[motorIndex];

    MotionExecutor& motionExecutor =
        *motionExecutors_[motorIndex];

    Homing& homing =
        *homings_[motorIndex];

    // --------------------------------------------
    // command + argument
    // --------------------------------------------

    int separator = line.indexOf(' ');

    String command;
    String argument;

    if (separator < 0)
    {
        command = line;
        argument = "";
    }
    else
    {
        command = line.substring(
            0,
            separator
        );

        argument = line.substring(
            separator + 1
        );

        argument.trim();
    }

    command.toLowerCase();

    // ============================================
    // ENABLE
    // ============================================

    if (command == "enable")
    {
        motor.enable();

        Serial.print("M");
        Serial.print(motorIndex + 1);
        Serial.println(" enabled");

        return;
    }

    // ============================================
    // DISABLE
    // ============================================

    if (command == "disable")
    {
        motor.disable();

        Serial.print("M");
        Serial.print(motorIndex + 1);
        Serial.println(" disabled");

        return;
    }

    // ============================================
    // START
    // ============================================

    if (command == "start")
    {
        if (motor.getSpeed() == 0)
        {
            Serial.println(
                "ERR speed = 0"
            );
            return;
        }

        if (!motor.isEnabled())
        {
            motor.enable();
        }

        motor.start();

        Serial.print("M");
        Serial.print(motorIndex + 1);
        Serial.println(" started");

        return;
    }

    // ============================================
    // STOP
    // ============================================

    if (command == "stop")
    {
        motor.stop();

        Serial.print("M");
        Serial.print(motorIndex + 1);
        Serial.println(" stopped");

        return;
    }

    // ============================================
    // SPEED
    // ============================================

    if (command == "speed")
    {
        long value = 0;

        if (!parseLongArgument(
                argument,
                value
            ))
        {
            Serial.println(
                "Usage: m1 speed <steps/s>"
            );

            return;
        }

        if (value < 0)
        {
            Serial.println(
                "ERR speed cannot be negative"
            );

            return;
        }

        motor.setSpeed(
            static_cast<uint32_t>(value)
        );

        Serial.print("M");
        Serial.print(motorIndex + 1);

        Serial.print(" speed = ");
        Serial.print(value);

        Serial.println(" steps/s");

        return;
    }

    // ============================================
    // DIRECTION
    // ============================================

    if (command == "dir")
    {
        if (argument == "0")
        {
            motor.setDirection(false);

            Serial.print("M");
            Serial.print(motorIndex + 1);
            Serial.println(" dir = 0");

            return;
        }

        if (argument == "1")
        {
            motor.setDirection(true);

            Serial.print("M");
            Serial.print(motorIndex + 1);
            Serial.println(" dir = 1");

            return;
        }

        Serial.println(
            "Usage: m1 dir <0|1>"
        );

        return;
    }

    // ============================================
    // RMS CURRENT
    // ============================================

    if (command == "rms")
    {
        long value = 0;

        if (!parseLongArgument(
                argument,
                value
            ))
        {
            Serial.println(
                "Usage: m1 rms <mA>"
            );

            return;
        }

        if (value <= 0 || value > 3000)
        {
            Serial.println(
                "ERR invalid RMS current"
            );

            return;
        }

        tmc.rms_current(
            static_cast<uint16_t>(value)
        );

        Serial.print("M");
        Serial.print(motorIndex + 1);

        Serial.print(" RMS = ");
        Serial.print(value);

        Serial.println(" mA");

        return;
    }

    // ============================================
    // MICROSTEPS
    // ============================================

    if (command == "microsteps")
    {
        long value = 0;

        if (!parseLongArgument(
                argument,
                value
            ))
        {
            Serial.println(
                "Usage: m1 microsteps <value>"
            );

            return;
        }

        switch (value)
        {
            case 0:
            case 2:
            case 4:
            case 8:
            case 16:
            case 32:
            case 64:
            case 128:
            case 256:
                break;

            default:
                Serial.println(
                    "ERR microsteps: "
                    "0,2,4,8,16,32,64,128,256"
                );

                return;
        }

        tmc.microsteps(
            static_cast<uint16_t>(value)
        );

        Serial.print("M");
        Serial.print(motorIndex + 1);

        Serial.print(" microsteps = ");
        Serial.println(value);

        return;
    }

    // ============================================
    // MODE
    // ============================================

    if (command == "mode")
    {
        String mode = argument;

        mode.toLowerCase();

        if (mode == "stealth")
        {
            tmc.en_spreadCycle(false);
            tmc.pwm_autoscale(true);

            Serial.print("M");
            Serial.print(motorIndex + 1);
            Serial.println(
                " mode = StealthChop"
            );

            return;
        }

        if (mode == "spread")
        {
            tmc.en_spreadCycle(true);

            Serial.print("M");
            Serial.print(motorIndex + 1);
            Serial.println(
                " mode = SpreadCycle"
            );

            return;
        }

        Serial.println(
            "Usage: m1 mode <stealth|spread>"
        );

        return;
    }

    // ============================================
    // HOME
    // ============================================

    if (command == "home")
    {
        if (homing.isActive())
        {
            Serial.println(
                "ERR homing already active"
            );
            return;
        }

        motionExecutor.stop();
        motor.stop();

        if (!motor.isEnabled())
        {
            motor.enable();
        }

        homing.home();

        Serial.print("M");
        Serial.print(motorIndex + 1);
        Serial.println(" homing started");

        return;
    }
    // ============================================
    // STATUS
    // ============================================

    if (command == "status")
    {
        printStatus(
            motorIndex + 1
        );

        return;
    }

    // ============================================
    // HELP
    // ============================================

    if (command == "help")
    {
        printHelp();
        return;
    }

    // ============================================
    // UNKNOWN
    // ============================================

    Serial.print("ERR unknown M");
    Serial.print(motorIndex + 1);
    Serial.print(" command: ");
    Serial.println(command);
}

bool SerialCommandHandler::trapeze(
    uint8_t motor,
    double distance,
    double time,
    double accelerationTime,
    double timeStep
) {
    // if (motor < 1 || motor > 2) {
    //     return false;
    // }

    // uint8_t motorIndex = motor - 1;

    // MotionExecutor* motionExecutor =
    //     motionExecutors_[motorIndex];

    // TrajectoryGenerator* trajectoryGenerator =
    //     trajectoryGenerators_[motorIndex];

    // MotorDriver* motorDriver =
    //     motors_[motorIndex];

    // if (motionExecutor == nullptr ||
    //     trajectoryGenerator == nullptr) {
    //     return false;
    // }

    // if (time <= 0.0 ||
    //     accelerationTime <= 0.0 ||
    //     accelerationTime > time / 2.0 ||
    //     timeStep <= 0.0) {
    //     return false;
    // }

    // if (!trajectoryGenerator->trapezoidalProfile(
    //         distance,
    //         time,
    //         accelerationTime,
    //         timeStep
    //     )) {
    //     return false;
    // }

    // trajectoryGenerator->convertToSteps();

    // motionExecutor->setTimeStep(timeStep);
    // motionExecutor->start(trajectoryGenerator->takeStepTrajectory(), timeStep);

    // if (!motorDriver->isEnabled()) {
    //     motorDriver->enable();
    // }

    return false;
}

bool SerialCommandHandler::cosine(
    uint8_t motor,
    double amplitude,
    double frequency,
    double duration,
    double timeStep
) {
    // uint8_t motorIndex = motor;

    // if (motor == 1) {
    //     motorIndex = 0;
    // } else if (motor == 2) {
    //     motorIndex = 1;
    // } else if (motor != 0) {
    //     return false;
    // }

    // MotionExecutor* motionExecutor =
    //     motionExecutors_[motorIndex];

    // TrajectoryGenerator* trajectoryGenerator =
    //     trajectoryGenerators_[motorIndex];

    // MotorDriver* motorDriver =
    //     motors_[motorIndex];

    // if (motionExecutor == nullptr ||
    //     trajectoryGenerator == nullptr) {
    //     return false;
    // }

    // if (amplitude < 0.0 ||
    //     frequency <= 0.0 ||
    //     duration <= 0.0 ||
    //     timeStep <= 0.0) {
    //     return false;
    // }

    // trajectoryGenerator->cosinusoidalTrajectory(
    //     amplitude,
    //     frequency,
    //     duration,
    //     timeStep
    // );
    // trajectoryGenerator->convertToSteps();

    // motionExecutor->setTimeStep(timeStep);
    // motionExecutor->start(
    //     trajectoryGenerator->takeStepTrajectory(),
    //     timeStep
    // );

    // if (!motorDriver->isEnabled()) {
    //     motorDriver->enable();
    // }

    return false;
}

bool SerialCommandHandler::A2B(
    double pointA_x,
    double pointA_y,
    double pointB_x,
    double pointB_y,
    double time,
    double timeStep
)
{
    return motionManager.planA2B(
        pointA_x,
        pointA_y,
        pointB_x,
        pointB_y,
        time,
        timeStep
    );
}
bool SerialCommandHandler::line(
    double ax,
    double ay,
    double bx,
    double by,
    double speed,
    double timeStep
)
{
    return motionManager.planLine(
        ax,
        ay,
        bx,
        by,
        speed,
        timeStep
    );
}

void SerialCommandHandler::handleTabletCommand(
    const String& line
)
{
    double x;
    double y;
    double pressure;
    int inside;

    int parsed = sscanf(
        line.c_str(),
        "tablet %lf %lf %lf %d",
        &x,
        &y,
        &pressure,
        &inside
    );

    if (parsed != 4)
    {
        Serial.println("ERR tablet");
        return;
    }

    motionManager.setLiveTarget(
        x,
        y,
        pressure,
        inside != 0
    );
}

void SerialCommandHandler::handleMotionCommand(
    String line
)
{
    line.trim();

    if (line.length() == 0)
    {
        Serial.println(
            "ERR missing motion command"
        );
        return;
    }

    // Nie przyjmujemy kolejnego ruchu,
    // gdy poprzedni jest jeszcze planowany.
    if (motionCommandPending_)
    {
        Serial.println(
            "ERR motion planning already pending"
        );
        return;
    }

    // Nie przyjmujemy nowego ruchu,
    // gdy silniki wykonują poprzednia trajektorie.
    if (
        motionExecutors_[0]->isActive() ||
        motionExecutors_[1]->isActive()
    )
    {
        Serial.println(
            "ERR motion already active"
        );
        return;
    }

    // Pierwsze slowo:
    // a2b / line

    int separator = line.indexOf(' ');

    String command;
    String arguments;

    if (separator < 0)
    {
        command = line;
        arguments = "";
    }
    else
    {
        command = line.substring(
            0,
            separator
        );

        arguments = line.substring(
            separator + 1
        );

        arguments.trim();
    }

    command.toLowerCase();

    // ============================================
    // A2B
    //
    // motion a2b Ax Ay Bx By time dt
    // ============================================

    if (command == "a2b")
    {
        double ax;
        double ay;
        double bx;
        double by;
        double time;
        double timeStep;

        int parsed = sscanf(
            arguments.c_str(),
            "%lf %lf %lf %lf %lf %lf",
            &ax,
            &ay,
            &bx,
            &by,
            &time,
            &timeStep
        );

        if (parsed != 6)
        {
            Serial.println(
                "Usage: motion a2b "
                "<Ax> <Ay> <Bx> <By> <time> <dt>"
            );
            return;
        }

        if (time <= 0.0)
        {
            Serial.println(
                "ERR time must be > 0"
            );
            return;
        }

        if (timeStep <= 0.0)
        {
            Serial.println(
                "ERR dt must be > 0"
            );
            return;
        }

        if (!motionManager.planA2B(
                ax,
                ay,
                bx,
                by,
                time,
                timeStep
            ))
        {
            Serial.println(
                "ERR could not queue A2B"
            );
            return;
        }

        motionCommandPending_ = true;
        motionCommandQueuedAt_ = millis();

        Serial.println(
            "OK A2B queued"
        );

        return;
    }

    // ============================================
    // LINE
    //
    // motion line Ax Ay Bx By speed dt
    // ============================================

    if (command == "line")
    {
        double ax;
        double ay;
        double bx;
        double by;
        double speed;
        double timeStep;

        int parsed = sscanf(
            arguments.c_str(),
            "%lf %lf %lf %lf %lf %lf",
            &ax,
            &ay,
            &bx,
            &by,
            &speed,
            &timeStep
        );

        if (parsed != 6)
        {
            Serial.println(
                "Usage: motion line "
                "<Ax> <Ay> <Bx> <By> <speed> <dt>"
            );
            return;
        }

        if (speed <= 0.0)
        {
            Serial.println(
                "ERR speed must be > 0"
            );
            return;
        }

        if (timeStep <= 0.0)
        {
            Serial.println(
                "ERR dt must be > 0"
            );
            return;
        }

        if (!motionManager.planLine(
                ax,
                ay,
                bx,
                by,
                speed,
                timeStep
            ))
        {
            Serial.println(
                "ERR could not queue LINE"
            );
            return;
        }

        motionCommandPending_ = true;
        motionCommandQueuedAt_ = millis();

        Serial.println(
            "OK LINE queued"
        );

        return;
    }

    Serial.print(
        "ERR unknown motion command: "
    );

    Serial.println(command);
}

void SerialCommandHandler::updatePendingMotion()
{
    if (!motionCommandPending_)
    {
        return;
    }

    if (motionManager.isTrajectoryReady())
    {
        if (!motionManager.startPreparedMotion())
        {
            Serial.println(
                "ERR motion start failed"
            );

            motionCommandPending_ = false;
            return;
        }

        Serial.println(
            "OK motion started"
        );

        motionCommandPending_ = false;
        return;
    }

    // Zabezpieczenie, gdyby planner zwrocil blad
    // i trajektoria nigdy nie przeszla w READY.
    if (
        millis() - motionCommandQueuedAt_ >
        10000
    )
    {
        Serial.println(
            "ERR motion planning timeout"
        );

        motionCommandPending_ = false;
    }
}

void SerialCommandHandler::stopAll()
{
    motionExecutors_[0]->stop();
    motionExecutors_[1]->stop();

    motors_[0]->stop();
    motors_[1]->stop();

    stopped_ = true;
}