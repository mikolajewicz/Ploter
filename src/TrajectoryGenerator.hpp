#include <vector>
#include <math.h>
#include <optional>


class TrajectoryGenerator
{
private:
    std::vector<double> trajectoryPoints;
    std::vector<int> stepTrajectory;
    double timeStep;
    int stepsPerRevolution = 1600; // Liczba kroków na pełny obrót ramienia
    size_t current_id = 0;
    
public:
    TrajectoryGenerator(int stepsPerRevolution);

    void setCurrentId(size_t id);
    void sinusoidalTrajectory(double amplitude, double frequency, double duration, double timeStep);

    bool getCurrentPosition(double& point);

    bool getNextPosition(double& point);

    void convertToSteps(std::vector<int>& stepTrajectory);
    int getStepsPerRevolution() const { return stepsPerRevolution; }


};



