kinematics::kinematics()
{
    // Constructor implementation (if needed)
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
        double x = x_vect[i];
        double y = y_vect[i];

        if (abs(x) > A4_width/2 - margin || abs(y) > A4_height/2 - margin)
        {
            // Invalid coordinates, return false
            return false;
        }
    }

    return true;
}

