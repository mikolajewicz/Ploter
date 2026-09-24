#pragma once

#include <cmath>
#include <vector>
#include <Arduino.h>

class Kinematics
{
public:
    Kinematics();
    bool compute();
    void setPath(std::vector<double>& x_vect, std::vector<double>& y_vect);
    void pathInput(double dt);
    bool A2B(
        double pointA_x,
        double pointA_y,
        double pointB_x,
        double pointB_y,
        double time,
        double dt,
        double accelerationTime);

    bool A2B(
        double pointA_x,
        double pointA_y,
        double pointB_x,
        double pointB_y,
        double time,
        double dt);

    std::vector<double> takeMotor1Trajectory(){return std::move(theta_1_vect);}
    std::vector<double> takeMotor2Trajectory(){return std::move(theta_2_vect);}

    bool line(
        double pointA_x,
        double pointA_y,
        double pointB_x,
        double pointB_y,
        double speed, // mm/s
        double timeStep = 0.01);

    bool forwardKinematics(double m1_angle, double m2_angle, double& x, double& y); // w stopniach

private:
    std::vector<double> x_vect;
    std::vector<double> y_vect;

    std::vector<double> theta_1_vect;
    std::vector<double> theta_2_vect;
    
};
