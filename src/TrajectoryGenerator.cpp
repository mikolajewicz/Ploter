#include "TrajectoryGenerator.hpp"
#include <cmath>

TrajectoryGenerator::TrajectoryGenerator(unsigned int stepsPerRevolution)
    : timeStep(0.0),
      current_id(0),
      stepsPerRevolution(stepsPerRevolution)
{
}

bool TrajectoryGenerator::trapezoidalProfile(double distance, double time, double acceleration, double timeStep) {
   
    if (time <= 0.0 || acceleration <= 0.0) {
        return false;
    }

    trajectoryPoints.clear();
    size_t samples = static_cast<size_t>(time / timeStep);

    double t_const_squared = time * time - (4 * distance / acceleration);

    if (t_const_squared < 0) {
        return false;
    }

    double t_const = std::sqrt(t_const_squared);
    double t_accel = 0.5 * (time - t_const);
    double v_max = (time - t_const) * acceleration / 2.0;

    for (double t = 0.0; t <= time; t += timeStep) {
        if (t <= t_accel) {
            trajectoryPoints.push_back(0.5 * acceleration * std::pow(t, 2));
        } else if (t <= t_accel + t_const) {
            trajectoryPoints.push_back(v_max * (t - t_accel) + 0.5 * acceleration * std::pow(t_accel, 2));
        } else {
            trajectoryPoints.push_back(distance - 0.5 * acceleration * std::pow(time - t, 2));
        }
    }

    return true;
}

void TrajectoryGenerator::sinusoidalTrajectory(double amplitude, double frequency, double duration, double timeStep)
{
    this->timeStep = timeStep;
    
    trajectoryPoints.clear();

    size_t samples = static_cast<size_t>(duration / timeStep);

    for (size_t i = 0; i <= samples; ++i) {
        double t = i * timeStep;
        double q = amplitude * sin(2.0 * M_PI * frequency * t);

        trajectoryPoints.push_back(q);
    }

}

bool TrajectoryGenerator::getNextPosition(double& point)
{
    if (current_id >= trajectoryPoints.size()) {
        return false;
    }

    point = trajectoryPoints[current_id++];
    return true;
}

bool TrajectoryGenerator::getCurrentPosition(double& point) {
    if (current_id >= trajectoryPoints.size()) {
        return false;
    }

    point = trajectoryPoints[current_id];
    return true;
}

void TrajectoryGenerator::convertToSteps(std::vector<int>& stepTrajectory) {
    stepTrajectory.clear();
    
    current_id = 0;
    double diffrence = 0;
    double residue = 0;
    
    for (size_t i = 0; i + 1 < trajectoryPoints.size(); ++i) {
        diffrence = (trajectoryPoints[i + 1] - trajectoryPoints[i]) / 360.0 * stepsPerRevolution + residue;
        int stepValue = static_cast<int>(diffrence);
        residue = diffrence - stepValue;
        
        stepTrajectory.push_back(stepValue);
    }
}   