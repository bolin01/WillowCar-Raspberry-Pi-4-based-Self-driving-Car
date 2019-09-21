#ifndef BICYCLE_K_H
#define BICYCLE_K_H

#include <vector>
#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <cmath>

namespace mpc_traj_follower {
    
struct BicycleKinematic
{
public:
    // Constructor and Destrucor
    BicycleKinematic(double Lf);
    BicycleKinematic();
    ~BicycleKinematic() {}

    // Define operator(). Object called by integrator.
    void operator() (const std::vector<double> &x , std::vector<double> &dxdt , const double t)
    {
        ComputeXDot(x, dxdt, t);
    }

    // Set input u. See comments in definition.
    double setInput(std::vector<double> steer, std::vector<double> acc, double t0, double t1);   

private:
    // Vehicle parameters
    double lf;

    // Integration parameters
    double dt;
    double t0;  // Integration start time
    double t1;  // Integration end time
    std::vector<double> steer_input;
    std::vector<double> acc_input;

    // Input limit
    std::vector<double> steer_limit;
    std::vector<double> acc_limit;

    // History of integration
    std::vector<std::vector<double>> hist;  // [[x_i, y_i, v_i, phi_i], ...]

    // Helper Functions
    double Rad2Deg(double ang) { return ang * 180.0 / M_PI; }  // Radian to degree
    void ComputeXDot(const std::vector<double> &x , std::vector<double> &dxdt , const double t);  // Called in operator ()
};

} /* end of namespace mpc_traj_follower */

#endif /* BICYCLE_6_H */