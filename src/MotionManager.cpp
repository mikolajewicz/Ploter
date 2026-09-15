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

    Kinematics& solver
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

      solver(solver)
{
}

bool MotionManager::A2B(
    double pointA_x, 
    double pointA_y, 
    double pointB_x, 
    double pointB_y,
    double time,
    double timeStep
){   
        if (!solver.A2B(
            pointA_x,
            pointA_y,
            pointB_x,
            pointB_y,
            time,
            timeStep)
        ) {
            return false;
        }

        if (!solver.compute()) {
            return false;
        }
        trajectoryGenerator1.setTrajectory(solver.takeMotor1Trajectory());
        trajectoryGenerator2.setTrajectory(solver.takeMotor2Trajectory());

        trajectoryGenerator1.convertToSteps();
        trajectoryGenerator2.convertToSteps();

        motionExecutor1.setTimeStep(timeStep);
        motionExecutor1.start(trajectoryGenerator1.takeStepTrajectory(), timeStep);

        motionExecutor2.setTimeStep(timeStep);
        motionExecutor2.start(trajectoryGenerator2.takeStepTrajectory(), timeStep);
        return true;
}

bool MotionManager::line(
    double pointA_x, 
    double pointA_y, 
    double pointB_x, 
    double pointB_y,
    double speed,
    double timeStep){
        if (!solver.line(
            pointA_x,
            pointA_y,
            pointB_x,
            pointB_y,
            speed,
            timeStep)
        ) {
            return false;
        }

        if (!solver.compute()) {
            return false;
        }

        trajectoryGenerator1.setTrajectory(solver.takeMotor1Trajectory());
        trajectoryGenerator2.setTrajectory(solver.takeMotor2Trajectory());

        trajectoryGenerator1.convertToSteps();
        trajectoryGenerator2.convertToSteps();

        motionExecutor1.setTimeStep(timeStep);
        motionExecutor1.start(trajectoryGenerator1.takeStepTrajectory(), timeStep);

        motionExecutor2.setTimeStep(timeStep);
        motionExecutor2.start(trajectoryGenerator2.takeStepTrajectory(), timeStep);
        return true;
}