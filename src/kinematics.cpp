#include "Kinematics.hpp"

Kinematics::Kinematics()
{
    // Constructor implementation (if needed)
}

void Kinematics::pathInput(double dt)
{
    double frequency = 0.1;
    for (size_t i = 0; i < 1/dt; ++i)
    {
        x_vect.push_back(std::sin(i * dt * 2 * M_PI * frequency) * 50.0); // Example: sine wave path
        y_vect.push_back(std::cos(i * dt * 2 * M_PI * frequency) * 50.0); // Example: cosine wave path
    }
}

void Kinematics::setPath(std::vector<double>& x_vect, std::vector<double>& y_vect)
{
    this->x_vect = x_vect;
    this->y_vect = y_vect;
}

bool Kinematics::compute()
{
    theta_1_vect.clear();
    theta_2_vect.clear();

    double A4_width = 210.0; // mm
    double A4_height = 297.0; // mm
    double margin = 10.0; // mm

    double l1 = 180.0; // Length of the first arm segment
    double l2 = 260.0; // Length of the second arm segment

    for (size_t i = 0; i < x_vect.size(); ++i)
    {

        double kx = x_vect[i];
        double ky = y_vect[i] + 150 + A4_height/2;

        if (abs(kx) > A4_width/2 - margin || ky > 150 + A4_height - margin || ky < 150 + margin)
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

bool Kinematics::A2B(
    double pointA_x,
    double pointA_y,
    double pointB_x,
    double pointB_y,
    double time,
    double dt){
    return A2B(
        pointA_x,
        pointA_y,
        pointB_x,
        pointB_y,
        time,
        dt,
        time / 3.0
    );
}


bool Kinematics::A2B(
    double pointA_x,
    double pointA_y,
    double pointB_x,
    double pointB_y,
    double time,
    double timeStep,
    double accelerationTime){
        double distance_x = pointB_x - pointA_x;
        double distance_y = pointB_y - pointA_y;

        if (time <= 0.0 ||
            accelerationTime <= 0.0 ||
            accelerationTime > time / 2.0 ||
            timeStep <= 0.0) {
                return false;
        }
        double sign_x = 1.0;
        double sign_y = 1.0;

        if (distance_x < 0.0) {
            sign_x = -1.0;
            distance_x = -distance_x;
        }

        if (distance_y < 0.0) {
            sign_y = -1.0;
            distance_y = -distance_y;
        }

        x_vect.clear();
        y_vect.clear();

        size_t samples =
        static_cast<size_t>(time / timeStep);

        double t_accel = accelerationTime;

        double t_const =
            time - 2.0 * accelerationTime;

        double acceleration_x =
            distance_x /
            (accelerationTime * (time - accelerationTime));

        double acceleration_y =
            distance_y /
            (accelerationTime * (time - accelerationTime));

        double v_max_x =
            acceleration_x * accelerationTime;

        double v_max_y =
            acceleration_y * accelerationTime;

        for (size_t i = 0; i <= samples; ++i) {
        double t = i * timeStep;

        if (t <= t_accel) {

            x_vect.push_back(
                pointA_x + 
                sign_x *
                0.5 *
                acceleration_x *
                t * t
            );

            y_vect.push_back(
                pointA_y + 
                sign_y *
                0.5 *
                acceleration_y *
                t * t
            );
        }
        else if (t <= t_accel + t_const) {

            x_vect.push_back(
                pointA_x +
                sign_x * (
                    v_max_x * (t - t_accel) +
                    0.5 *
                    acceleration_x *
                    t_accel *
                    t_accel
                )
            );

            y_vect.push_back(
                pointA_y +
                sign_y * (
                    v_max_y * (t - t_accel) +
                    0.5 *
                    acceleration_y *
                    t_accel *
                    t_accel
                )
            );
        }
        else {

            x_vect.push_back(
                pointA_x +
                sign_x * (
                    distance_x -
                    0.5 *
                    acceleration_x *
                    (time - t) *
                    (time - t)
                )
            );

            y_vect.push_back(
                pointA_y +
                sign_y * (
                    distance_y -
                    0.5 *
                    acceleration_y *
                    (time - t) *
                    (time - t)
                )
            );
        }
        
    }
    return true;
}

bool Kinematics::line(
    double pointA_x,
    double pointA_y,
    double pointB_x,
    double pointB_y,
    double speed,
    double timeStep
)
{
    if (speed <= 0.0 || timeStep <= 0.0) {
        return false;
    }

    double dx = pointB_x - pointA_x;
    double dy = pointB_y - pointA_y;

    double distance = std::hypot(dx, dy);

    if (distance == 0.0) {
        return false;
    }

    double time = distance / speed;

    size_t samples =
        static_cast<size_t>(time / timeStep);

    x_vect.clear();
    y_vect.clear();

    for (size_t i = 0; i <= samples; ++i) {
        double t = i * timeStep;

        double progress = t / time;

        x_vect.push_back(
            pointA_x + dx * progress
        );

        y_vect.push_back(
            pointA_y + dy * progress
        );
    }

    // jesli timeStep nie trafia idealnie w koniec
    if (samples * timeStep < time) {
        x_vect.push_back(pointB_x);
        y_vect.push_back(pointB_y);
    }

    return true;
}

bool Kinematics::forwardKinematics(
    double m1_angle,
    double m2_angle,
    double& x,
    double& y
)
{
    constexpr double l1 = 180.0;
    constexpr double l2 = 260.0;
    constexpr double d  = 150.0;

    // stopnie -> radiany
    double q1 = m1_angle * M_PI / 180.0;
    double q2 = m2_angle * M_PI / 180.0;

    // Punkt D
    double rD_x = l1 * std::cos(q1) - d / 2.0;
    double rD_y = l1 * std::sin(q1);

    // Punkt C
    double rC_x = l1 * std::cos(q2) + d / 2.0;
    double rC_y = l1 * std::sin(q2);

    // Środek odcinka CD
    double S_CD_x = (rD_x + rC_x) / 2.0;
    double S_CD_y = (rD_y + rC_y) / 2.0;

    // Wektor D -> C
    double CD_x = rC_x - rD_x;
    double CD_y = rC_y - rD_y;

    // Długość CD
    double CD = std::sqrt(
        CD_x * CD_x +
        CD_y * CD_y
    );

    // Sprawdzenie, czy drugi człon może połączyć C i D
    double hSquared =
        l2 * l2 -
        (CD / 2.0) * (CD / 2.0);

    if (hSquared < 0.0)
    {
        return false;
    }

    double alpha = std::atan2(
        CD_y,
        CD_x
    );

    double h = std::sqrt(hSquared);

    // Wektor prostopadły
    double h_v_x = -h * std::sin(alpha);
    double h_v_y =  h * std::cos(alpha);

    // Punkt końcowy K
    x = S_CD_x + h_v_x;
    y = S_CD_y + h_v_y;

    return true;
}

    
