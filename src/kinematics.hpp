#include <cmath>
#include <vector>

class kinematics
{
public:
    kinematics();
    bool compute();
    bool setPath(std::vector<double>& x_vect, std::vector<double>& y_vect);

private:
    std::vector<double> x_vect;
    std::vector<double> y_vect;

    std::vector<double> theta_1_vect;
    std::vector<double> theta_2_vect;
};
