#include "MotionManager.hpp"

MotionManager::MotionManager(
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

    Kinematics& solver,

    PenController& pen
)
    : motor1(motor1),
      tmc1(tmc1),
      motionExecutor1(motionExecutor1),
      trajectoryGenerator1(trajectoryGenerator1),
      homing1(homing1),

      motor2(motor2),
      tmc2(tmc2),
      motionExecutor2(motionExecutor2),
      trajectoryGenerator2(trajectoryGenerator2),
      homing2(homing2),

      solver(solver),
      pen(pen)
{
}

bool MotionManager::A2B(
    double pointA_x,
    double pointA_y,
    double pointB_x,
    double pointB_y,
    double time,
    double timeStep
)
{
    if (!solver.A2B(
            pointA_x,
            pointA_y,
            pointB_x,
            pointB_y,
            time,
            timeStep
        )) {
        return false;
    }

    if (!solver.compute()) {
        return false;
    }

    trajectoryGenerator1.takeTrajectory(
        solver.takeMotor1Trajectory()
    );

    trajectoryGenerator2.takeTrajectory(
        solver.takeMotor2Trajectory()
    );

    trajectoryGenerator1.convertToSteps();
    trajectoryGenerator2.convertToSteps();

    std::vector<int> prepared1 =
    trajectoryGenerator1.takeStepTrajectory();

    std::vector<int> prepared2 =
        trajectoryGenerator2.takeStepTrajectory();

    xSemaphoreTake(
        trajectoryMutex,
        portMAX_DELAY
    );

    nextTrajectory1 = std::move(prepared1);
    nextTrajectory2 = std::move(prepared2);

    nextTimeStep = timeStep;
    trajectoryReady = true;

    xSemaphoreGive(trajectoryMutex);

    return true;
}

bool MotionManager::startPreparedMotion()
{
    std::vector<int> trajectory1;
    std::vector<int> trajectory2;

    double timeStep;

    xSemaphoreTake(
        trajectoryMutex,
        portMAX_DELAY
    );

    if (!trajectoryReady) {
        xSemaphoreGive(trajectoryMutex);
        return false;
    }

    trajectory1 = std::move(nextTrajectory1);
    trajectory2 = std::move(nextTrajectory2);

    timeStep = nextTimeStep;

    trajectoryReady = false;

    xSemaphoreGive(trajectoryMutex);

    motionExecutor1.start(
        std::move(trajectory1),
        timeStep
    );

    motionExecutor2.start(
        std::move(trajectory2),
        timeStep
    );

    return true;
}

bool MotionManager::isTrajectoryReady()
{
    xSemaphoreTake(
        trajectoryMutex,
        portMAX_DELAY
    );

    bool ready = trajectoryReady;

    xSemaphoreGive(trajectoryMutex);

    return ready;
}

bool MotionManager::beginPlanner()
{
    motionQueue = xQueueCreate(
        4,
        sizeof(MotionRequest)
    );

    if (motionQueue == nullptr) {
        return false;
    }

    trajectoryMutex = xSemaphoreCreateMutex();

    if (trajectoryMutex == nullptr) {
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        plannerTaskEntry,
        "MotionPlanner",
        8192,
        this,
        1,
        &plannerTaskHandle,
        0
    );

    return result == pdPASS;
}

void MotionManager::plannerTaskEntry(void* parameter)
{
    MotionManager* manager =
        static_cast<MotionManager*>(parameter);

    manager->plannerTask();
}

void MotionManager::plannerTask()
{
    MotionRequest request;

    while (true)
    {
        if (xQueueReceive(
                motionQueue,
                &request,
                portMAX_DELAY
            ) != pdTRUE)
        {
            continue;
        }

        bool success = false;

        switch (request.type)
        {
            case MotionType::A2B:
                success = A2B(
                    request.pointA_x,
                    request.pointA_y,
                    request.pointB_x,
                    request.pointB_y,
                    request.value,
                    request.timeStep
                );
                break;

            case MotionType::LINE:
                success = line(
                    request.pointA_x,
                    request.pointA_y,
                    request.pointB_x,
                    request.pointB_y,
                    request.value,
                    request.timeStep
                );
                break;

                
        }

        if (!success)
        {
            Serial.println("Motion planning failed");
        }
    }
}

bool MotionManager::planA2B(
    double pointA_x,
    double pointA_y,
    double pointB_x,
    double pointB_y,
    double time,
    double timeStep
)
{
    if (motionQueue == nullptr) {
        return false;
    }

    MotionRequest request;

    request.type = MotionType::A2B;

    request.pointA_x = pointA_x;
    request.pointA_y = pointA_y;

    request.pointB_x = pointB_x;
    request.pointB_y = pointB_y;

    request.value = time;
    request.timeStep = timeStep;

    return xQueueSend(
        motionQueue,
        &request,
        0
    ) == pdTRUE;
}

bool MotionManager::line(
    double Ax,
    double Ay,
    double Bx,
    double By,
    double speed,
    double timeStep
)
{
    if (!solver.line(
            Ax,
            Ay,
            Bx,
            By,
            speed,
            timeStep
        ))
    {
        Serial.println("solver.line FAILED");
        return false;
    }

    if (!solver.compute())
    {
        Serial.println("solver.compute FAILED");
        return false;
    }

    trajectoryGenerator1.takeTrajectory(
        solver.takeMotor1Trajectory()
    );

    trajectoryGenerator2.takeTrajectory(
        solver.takeMotor2Trajectory()
    );

    trajectoryGenerator1.convertToSteps();
    trajectoryGenerator2.convertToSteps();

    std::vector<int> prepared1 =
        trajectoryGenerator1.takeStepTrajectory();

    std::vector<int> prepared2 =
        trajectoryGenerator2.takeStepTrajectory();

    xSemaphoreTake(
        trajectoryMutex,
        portMAX_DELAY
    );

    nextTrajectory1 = std::move(prepared1);
    nextTrajectory2 = std::move(prepared2);

    nextTimeStep = timeStep;
    trajectoryReady = true;

    xSemaphoreGive(trajectoryMutex);

    // Serial.println("LINE READY");

    return true;
}

bool MotionManager::planLine(
    double pointA_x,
    double pointA_y,
    double pointB_x,
    double pointB_y,
    double speed,
    double timeStep
)
{
    if (motionQueue == nullptr) {
        return false;
    }

    MotionRequest request;

    request.type = MotionType::LINE;

    request.pointA_x = pointA_x;
    request.pointA_y = pointA_y;
    request.pointB_x = pointB_x;
    request.pointB_y = pointB_y;

    request.value = speed;
    request.timeStep = timeStep;

    return xQueueSend(
        motionQueue,
        &request,
        0
    ) == pdTRUE;

}

void MotionManager::setLiveTarget(
    double x,
    double y,
    double pressure,
    bool inside
)
{
    liveTargetX = x;
    liveTargetY = y;

    livePressure = pressure;
    liveTargetInside = inside;

    liveTargetReceived = true;
    liveTargetLastUpdate = millis();

    if (!inside)
    {
        pen.up();
        return;
    }

    pen.setPressure(pressure);
}


bool MotionManager::calculateCurrentPosition(){
    double m1_steps = motor1.getStepCount();
    double m2_steps = motor2.getStepCount();

    unsigned int stepsPerRev = trajectoryGenerator1.getStepsPerRev();

    double m1_angle = m1_steps / stepsPerRev * 360;
    double m2_angle = m2_steps / stepsPerRev * 360;

    return solver.forwardKinematics(m1_angle, m2_angle, current_x, current_y);

  
}

bool MotionManager::followTarget()
{
    if (
        !liveTargetReceived ||
        !liveTargetInside ||
        millis() - liveTargetLastUpdate > 500
    )
    {
        current_speed = 0.0;
        line_vect_x = 0.0;
        line_vect_y = 0.0;

        return false;
    }

    double mass = 1.0;
    double stiffness = 100.0;
    double viscocity = 20.0;
    double move_time = 0.01;

    if (!calculateCurrentPosition())
    {
        return false;
    }

    constexpr double X_MAX = 138.5;
    constexpr double Y_MAX = 95.0;
    constexpr double MAX_SPEED = 300.0; // mm/s
    constexpr double MAX_ACCELERATION = 800.0; // mm/s^2

    double target_x = constrain(
        liveTargetX,
        -X_MAX,
        X_MAX
    );

    double target_y = constrain(
        liveTargetY,
        -Y_MAX,
        Y_MAX
    );

    double previous_distance =
        std::hypot(
            line_vect_x,
            line_vect_y
        );

    double speed_x = 0.0;
    double speed_y = 0.0;

    if (previous_distance > 0.000001)
    {
        speed_x =
            current_speed
            * line_vect_x
            / previous_distance;

        speed_y =
            current_speed
            * line_vect_y
            / previous_distance;
    }

    double distance_x =
        target_x - current_x;

    double distance_y =
        target_y - current_y;

    double distance =
        std::hypot(
            distance_x,
            distance_y
        );

    if (distance < 0.5)
    {
        current_speed = 0.0;
        line_vect_x = 0.0;
        line_vect_y = 0.0;

        return false;
    }

    double acceleration_x =
    (
        -viscocity * speed_x
        + stiffness * distance_x
    )
    / mass;

    double acceleration_y =
        (
            -viscocity * speed_y
            + stiffness * distance_y
        )
        / mass;


// --------------------------------------------------
// Limit przyspieszenia
// --------------------------------------------------

    
    double acceleration =
        std::hypot(
            acceleration_x,
            acceleration_y
        );

    if (acceleration > MAX_ACCELERATION)
    {
        double scale =
            MAX_ACCELERATION / acceleration;

        acceleration_x *= scale;
        acceleration_y *= scale;
    }


    // --------------------------------------------------
    // Aktualizacja prędkości
    // --------------------------------------------------

    speed_x +=
        acceleration_x * move_time;

    speed_y +=
        acceleration_y * move_time;


    // --------------------------------------------------
    // Limit prędkości
    // --------------------------------------------------

    double speed =
        std::hypot(
            speed_x,
            speed_y
        );

    if (speed > MAX_SPEED)
    {
        double scale =
            MAX_SPEED / speed;

        speed_x *= scale;
        speed_y *= scale;
    }

    current_speed =
        std::hypot(
            speed_x,
            speed_y
        );

    double new_distance_x =
        speed_x * move_time;

    double new_distance_y =
        speed_y * move_time;

    double next_x = constrain(
        current_x + new_distance_x,
        -X_MAX,
        X_MAX
    );

    double next_y = constrain(
        current_y + new_distance_y,
        -Y_MAX,
        Y_MAX
    );

    static uint32_t lastDebug = 0;

    new_distance_x =
        next_x - current_x;

    new_distance_y =
        next_y - current_y;

    if (
        std::hypot(
            new_distance_x,
            new_distance_y
        ) < 0.001
    )
    {
        current_speed = 0.0;
        line_vect_x = 0.0;
        line_vect_y = 0.0;

        return false;
    }

    line_vect_x = new_distance_x;
    line_vect_y = new_distance_y;

    return line(
        current_x,
        current_y,
        next_x,
        next_y,
        current_speed,
        0.0005
    );
}












