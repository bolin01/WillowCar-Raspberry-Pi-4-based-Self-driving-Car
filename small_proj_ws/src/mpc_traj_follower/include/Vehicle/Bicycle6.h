#include <vector>
#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <cmath>

namespace mpc_traj_follower {
    
class Bicycle6
{
public:
    // Constructor and Destrucor
    Bicycle6(double m, double W, double Nw, double f, double Iz, \
    double a, double b, double By, double Cy, double Dy, \
    double Ey, double Shy, double Svy);
    Bicycle6();
    ~Bicycle6() {}

    // Define operator(). Object called by integrator.
    void operator() (const std::vector<double> &x , std::vector<double> &dxdt , const double t)
    {
        compute_x_dot(x, dxdt, t);
    }

    // Set input u. See comments in definition.
    double set_input(std::vector<double> steer, std::vector<double> force, double t0, double t1);   

private:
    // Vehicle parameters
    double m;
    double W; 
    double Nw;
    double f;
    double Iz;
    double a;
    double b;
    double By;
    double Cy; 
    double Dy;
    double Ey; 
    double Shy;
    double Svy;

    // Integration parameters
    double dt;
    double t0;  // Integration start time
    double t1;  // Integration end time
    std::vector<double> steer_input;
    std::vector<double> force_input;

    // Input limit
    std::vector<double> steer_limit;
    std::vector<double> force_limit;

    // History of integration
    std::vector<std::vector<double>> hist;  // [[x_i, u_i, y_i, v_i, phi_i, r_i],...]

    // Helper Functions
    double rad2deg(double ang) {return ang*180.0/M_PI;}  // Radian to degree
    void compute_x_dot(const std::vector<double> &x , std::vector<double> &dxdt , const double t);  // Called in operator ()
};
}
