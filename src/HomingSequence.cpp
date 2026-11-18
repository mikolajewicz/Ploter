#include "HomingSequence.hpp"

#include "MotorDriver.hpp"
#include "MotionExecutor.hpp"
#include "TrajectoryGenerator.hpp"
#include "Homing.hpp"
#include "MotionManager.hpp"
#include "PenController.hpp"
#include "CommunicationManager.hpp" // sendResponse()

#include <cmath>

namespace
{
    constexpr double HOME_ANGLE_M1 = 141.866 - 180.0;
    constexpr double HOME_ANGLE_M2 = 180.0 - 141.866;

    constexpr double HOME_OFFSET_M1 = 101.0 + HOME_ANGLE_M1;
    constexpr double HOME_OFFSET_M2 = -101.0 + HOME_ANGLE_M2;

    constexpr double HOME_OFFSET_TIME = 2.0;
    constexpr double HOME_OFFSET_ACCEL_TIME = 0.5;
}

HomingSequence::HomingSequence(
    MotorDriver& motor1,
    TMC2209Stepper& tmc1,
    MotionExecutor& executor1,
    TrajectoryGenerator& generator1,
    Homing& homing1,

    MotorDriver& motor2,
    TMC2209Stepper& tmc2,
    MotionExecutor& executor2,
    TrajectoryGenerator& generator2,
    Homing& homing2,

    MotionManager& motionManager,
    PenController& pen,
    uint32_t stepsPerRevolution,
    uint16_t initialRmsCurrent,
    double timeStep,
    bool bypassHome
)
    : motor1_(motor1),
      tmc1_(tmc1),
      executor1_(executor1),
      generator1_(generator1),
      homing1_(homing1),

      motor2_(motor2),
      tmc2_(tmc2),
      executor2_(executor2),
      generator2_(generator2),
      homing2_(homing2),

      motionManager_(motionManager),
      pen_(pen),
      stepsPerRevolution_(stepsPerRevolution),
      initialRmsCurrent_(initialRmsCurrent),
      timeStep_(timeStep),
      bypassHome_(bypassHome)
{
}

bool HomingSequence::isActive() const
{
    return state_ >= 1 && state_ <= 3;
}

bool HomingSequence::isReady() const
{
    return state_ == 4;
}

int HomingSequence::state() const
{
    return state_;
}

bool HomingSequence::request()
{
    if (isActive())
    {
        return false;
    }

    // Tak jak w dotychczasowym requestHomeSequence().
    pen_.up();
    start();
    return true;
}

void HomingSequence::start()
{
    motionManager_.clearLiveTarget();

    executor1_.stop();
    executor2_.stop();

    motor1_.stop();
    motor2_.stop();

    if (bypassHome_)
    {
        motor1Homed_ = true;
        motor2Homed_ = true;

        motor1_.enable();
        motor2_.enable();

        motor1_.setStepCount(angleToSteps(HOME_ANGLE_M1));
        motor2_.setStepCount(angleToSteps(HOME_ANGLE_M2));

        state_ = 4;
        sendResponse("HOME sequence finished");
        return;
    }

    motor1Homed_ = false;
    motor2Homed_ = false;

    motor1_.enable();
    motor2_.enable();

    homing1_.home();
    homing2_.home();

    state_ = 1;
    sendResponse("HOME sequence started");
}

void HomingSequence::update()
{
    // Kolejność taka sama jak w dotychczasowym loop().
    const bool homing1Finished = homing1_.update();
    const bool homing2Finished = homing2_.update();

    switch (state_)
    {
        case 0:
        case 4:
            break;

        case 1:
        {
            if (homing1Finished)
            {
                motor1Homed_ = true;
                Serial.println("M1 homed");
            }

            if (homing2Finished)
            {
                motor2Homed_ = true;
                Serial.println("M2 homed");
            }

            if (motor1Homed_ && motor2Homed_)
            {
                Serial.println("Both motors homed");
                state_ = 2;
            }
            break;
        }

        case 2:
        {
            const bool ok1 = generator1_.trapezoidalProfile(
                HOME_OFFSET_M1,
                HOME_OFFSET_TIME,
                HOME_OFFSET_ACCEL_TIME,
                timeStep_
            );

            const bool ok2 = generator2_.trapezoidalProfile(
                HOME_OFFSET_M2,
                HOME_OFFSET_TIME,
                HOME_OFFSET_ACCEL_TIME,
                timeStep_
            );

            if (!ok1 || !ok2)
            {
                Serial.println("HOME offset trajectory ERROR");
                state_ = 99;
                break;
            }

            generator1_.convertToSteps();
            generator2_.convertToSteps();

            executor1_.setTimeStep(timeStep_);
            executor2_.setTimeStep(timeStep_);

            executor1_.start(
                generator1_.takeStepTrajectory(), timeStep_
            );
            executor2_.start(
                generator2_.takeStepTrajectory(), timeStep_
            );

            Serial.println("Moving from home to zero");
            state_ = 3;
            break;
        }

        case 3:
        {
            if (!executor1_.isActive() && !executor2_.isActive())
            {
                state_ = 4;

                motor1_.setStepCount(angleToSteps(HOME_ANGLE_M1));
                motor2_.setStepCount(angleToSteps(HOME_ANGLE_M2));

                tmc1_.rms_current(initialRmsCurrent_);
                tmc2_.rms_current(initialRmsCurrent_);

                sendResponse("HOME sequence finished");
            }
            break;
        }

        case 99:
            motor1_.stop();
            motor2_.stop();
            executor1_.stop();
            executor2_.stop();
            break;
    }
}

void HomingSequence::abort()
{
    if (!isActive())
    {
        return;
    }

    homing1_.abort();
    homing2_.abort();

    motor1Homed_ = false;
    motor2Homed_ = false;
    state_ = 0;

    sendResponse("HOME ABORTED");
}

void HomingSequence::reset()
{
    // Po wywołaniu stopPlotter() z DISABLE.
    if (isActive())
    {
        homing1_.abort();
        homing2_.abort();
    }

    motor1Homed_ = false;
    motor2Homed_ = false;
    state_ = 0;
}

int64_t HomingSequence::angleToSteps(double angle) const
{
    return llround(
        angle / 360.0 * static_cast<double>(stepsPerRevolution_)
    );
}
