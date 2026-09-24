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

    Serial.println("LINE READY");

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

    solver.forwardKinematics(m1_angle, m2_angle, current_x, current_y);

    return true;
}











