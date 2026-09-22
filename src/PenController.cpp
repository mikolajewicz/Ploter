#include "PenController.hpp"

PenController::PenController(
    uint8_t pin,
    int upAngle,
    int downAngle
)
    : pin(pin),
      upAngle(upAngle),
      downAngle(downAngle)
{
}

void PenController::begin()
{
    servo.setPeriodHertz(50);

    servo.attach(
        pin,
        500,
        2400
    );

    up();
}

void PenController::up()
{
    servo.write(upAngle);
    penDown = false;
}

void PenController::down()
{
    servo.write(downAngle);
    penDown = true;
}

void PenController::setPressure(double pressure)
{
    if (pressure > PRESSURE_THRESHOLD)
    {
        if (!penDown)
        {
            down();
        }
    }
    else
    {
        if (penDown)
        {
            up();
        }
    }
}

bool PenController::isDown() const
{
    return penDown;
}