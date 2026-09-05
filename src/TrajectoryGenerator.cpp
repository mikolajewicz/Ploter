#include "TrajectoryGenerator.hpp"

TrajectoryGenerator::TrajectoryGenerator(int stepsPerRevolution)
    : timeStep(0.0),
      current_id(0),
      stepsPerRevolution(stepsPerRevolution)
{
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