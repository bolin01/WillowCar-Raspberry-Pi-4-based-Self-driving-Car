#include <Vehicle/BicycleKinematic.h>

namespace mpc_traj_follower {

BicycleKinematic::BicycleKinematic(double Lf):
    lf(Lf)
{
    steer_limit = {-0.5, 0.5};
    acc_limit = {-6, 6};
    dt = 0.00;
    t0 = 0.00;
    t1 = 0.00;
}

BicycleKinematic::BicycleKinematic()
{
    // Default value in assignment
    steer_limit = {-0.5, 0.5};
    acc_limit = {-6, 6};
    dt = 0.00;
    t0 = 0.00;
    t1 = 0.00;
    lf = 2.5;
}

double BicycleKinematic::setInput(std::vector<double> steer, std::vector<double> acc, double t0, double t1)
{
    /* ODEINT does not support control input.
       The input here is stores in system Bicycle6.
       It is used in method compute_x_dot */
       
    assert(steer.size() == acc.size());

    steer_input.clear();
    acc_input.clear();

    // Make sure input never exceeds limit
    for (auto ang: steer)
    {
        if (ang > steer_limit[1])
            steer_input.push_back(steer_limit[1]);
        else if (ang < steer_limit[0])
            steer_input.push_back(steer_input[0]);
        else
            steer_input.push_back(ang);
    }
    for (auto ai: acc)
    {
        if (ai > acc_limit[1])
            acc_input.push_back(acc_limit[1]);
        else if (ai < acc_limit[0])
            acc_input.push_back(acc_limit[0]);
        else
            acc_input.push_back(ai);
    }

    // Set time
    this->t0 = t0;
    this->t1 = t1;
    this->dt = (t1-t0)/(steer.size()-1);
    return this->dt;
}

void BicycleKinematic::ComputeXDot(const std::vector<double> &x , std::vector<double> &dxdt , const double t)
{
    // x = [x_t, y_t, v_t, phi_t]
    double x_t = x[0], y_t = x[1], v_t = x[2], phi_t = x[3];
    double steer_t, acc_t;

    // Get the input from input vector. 0th order approximation in integration interval.
    int idx = (t - t0) / dt;
    assert(idx < steer_input.size());
    steer_t = steer_input[idx];
    acc_t = acc_input[idx];

    // compute dx/dt
    dxdt[0] = v_t*cos(phi_t);
    dxdt[1] = v_t*sin(phi_t);
    dxdt[2] = acc_t;
    dxdt[3] = v_t/lf*steer_t;
}

} /* end of namespace mpc_traj_follower */