#include "kinematics.hpp"

kinematics::kinematics()
{
    // Constructor implementation (if needed)
}

void kinematics::pathInput(double dt)
{
    double frequency = 0.1;
    for (size_t i = 0; i < 1/dt; ++i)
    {
        x_vect.push_back(std::sin(i * dt * 2 * M_PI * frequency) * 50.0); // Example: sine wave path
        y_vect.push_back(std::cos(i * dt * 2 * M_PI * frequency) * 50.0); // Example: cosine wave path
    }
}

bool kinematics::setPath(std::vector<double>& x_vect, std::vector<double>& y_vect)
{
    this->x_vect = x_vect;
    this->y_vect = y_vect;
    return true;
}

bool kinematics::compute()
{
    theta_1_vect.clear();
    theta_2_vect.clear();

    double A4_width = 210.0; // mm
    double A4_height = 210.0; // mm
    double margin = 10.0; // mm

    for (size_t i = 0; i < x_vect.size(); ++i)
    {
        double l1 = 100.0; // Length of the first arm segment
        double l2 = 100.0; // Length of the second arm segment

        double kx = x_vect[i];
        double ky = y_vect[i];

        if (abs(kx) > A4_width/2 - margin || abs(ky) > A4_height/2 - margin)
        {
            Serial.println(
                "Invalid path point: (" + String(kx) + ", " + String(ky) + ")"
            );
            
            return false;
        }

        // współrzędne silników
        double xA = -75; double yA = 0;
        double xB = 75; double yB = 0;

        // Odległości od baz do punktu K
        double a = std::hypot(kx - xA, ky - yA);   // |AK|
        double b = std::hypot(kx - xB, ky - yB);   // |BK|

        // Kierunki do końcówki
        double alpha1 = std::atan2(ky - yA, kx - xA);
        double beta1  = std::atan2(ky - yB, kx - xB);
       
        // Cosinusy do acos (prawo cosinusów)
        double cA = (l1*l1 + a*a - l2*l2) / (2*l1*a);
        double cB = (l1*l1 + b*b - l2*l2) / (2*l1*b);

        double alpha2 = std::acos(cA);
        double beta2  = std::acos(cB);

        double Q1 = alpha1 + alpha2;
        double Q2 = beta1  - beta2;

        theta_1_vect.push_back(Q1 * 180 / M_PI);
        theta_2_vect.push_back(Q2 * 180 / M_PI); // Convert to degrees

        
    }


    return true;
}

