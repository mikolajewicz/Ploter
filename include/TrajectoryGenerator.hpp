#pragma once

#include <vector>
#include <math.h>
#include <optional>


class TrajectoryGenerator
{
private:
    std::vector<double> trajectoryPoints;
    std::vector<int> stepTrajectory;
    double timeStep;
    unsigned int stepsPerRevolution = 1600; // Liczba kroków na pełny obrót ramienia
    size_t current_id = 0;
    
public:
    TrajectoryGenerator(unsigned int stepsPerRevolution);

    void setCurrentId(size_t id) {
        current_id = id;
    }
    void cosinusoidalTrajectory(double amplitude, double frequency, double duration, double timeStep);

    bool getCurrentPosition(double& point);

    bool getNextPosition(double& point);

    void convertToSteps();

    bool trapezoidalProfile(double distance, double time, double acceleration, double timeStep);

    void takeTrajectory(std::vector<double>&& trajectoryPoints);

    std::vector<int> takeStepTrajectory() {
        return std::move(stepTrajectory);
    }
};



