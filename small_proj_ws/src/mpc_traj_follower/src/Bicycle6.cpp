#include <Vehicle/Bicycle6.h>

namespace mpc_traj_follower {

Bicycle6::Bicycle6(double m, double W, double Nw, double f, double Iz, \
    double a, double b, double By, double Cy, double Dy, \
    double Ey, double Shy, double Svy):
    m(m), W(W), Nw(Nw), f(f), Iz(Iz), a(a), b(b), By(By), Cy(Cy), Dy(Dy), Ey(Ey), Shy(Shy), Svy(Svy)
{
    steer_limit = {-0.5, 0.5};
    force_limit = {-10000, 5000};
    dt = 0.00;
    t0 = 0.00;
    t1 = 0.00;
}

Bicycle6::Bicycle6()
{
    // Default value in assignment
    m = 1400;
    W = 13720;
    Nw = 2;
    f = 0.01;
    Iz = 2667;
    a = 1.35;
    b = 1.45;
    By = 0.27;
    Cy = 1.2;
    Dy = 2921;
    Ey = -1.6;
    Shy = 0;
    Svy = 0;
    steer_limit = {-0.5, 0.5};
    force_limit = {-10000, 5000};
    dt = 0.00;
    t0 = 0.00;
    t1 = 0.00;
}

double Bicycle6::set_input(std::vector<double> steer, std::vector<double> force, double t0, double t1)
{
    /* ODEINT does not support control input.
       The input here is stores in system Bicycle6.
       It is used in method compute_x_dot */
       
    assert(steer.size() == force.size());

    steer_input.clear();
    force_input.clear();

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
    for (auto fi: force)
    {
        if (fi > force_limit[1])
            force_input.push_back(force_limit[1]);
        else if (fi < force_limit[0])
            force_input.push_back(force_limit[0]);
        else
            force_input.push_back(fi);
    }

    // Set time
    this->t0 = t0;
    this->t1 = t1;
    this->dt = (t1-t0)/(steer.size()-1);
    return this->dt;
}

void Bicycle6::compute_x_dot(const std::vector<double> &x , std::vector<double> &dxdt , const double t)
{
    double x_i = x[0], u_i = x[1], y_i = x[2], v_i = x[3], phi_i = x[4], r_i = x[5];
    double alpha_f, alpha_r, steer_i, force_i;

    // Get the input from input vector. 0th order approximation in integration interval.
    int idx = (t - t0) / dt;
    assert(idx < steer_input.size());
    steer_i = steer_input[idx];
    force_i = force_input[idx];

    // Compute alpha_f and alpha_r
    alpha_f = steer_i - atan2(v_i+a*r_i, u_i);
    alpha_r = -atan2(v_i-b*r_i, u_i);
    alpha_f = rad2deg(alpha_f);
    alpha_r = rad2deg(alpha_r);

    // Compute phi_yf and phi_yr
    double phi_yf, phi_yr;
    phi_yf = (1-Ey)*(alpha_f+Shy) + Ey/By*atan(By*(alpha_f+Shy));
    phi_yr = (1-Ey)*(alpha_r+Shy) + Ey/By*atan(By*(alpha_r+Shy));

    // Compute Fyf and Fyr
    double Fyf, Fyr;
    Fyf = Dy*sin(Cy*atan(By*phi_yf)) + Svy;
    Fyr = Dy*sin(Cy*atan(By*phi_yr)) + Svy;

    // Compute time derivative
    dxdt[0] = u_i*cos(phi_i) - v_i*sin(phi_i);
    dxdt[1] = 1/m*(-f*W+Nw*force_i-Fyf*sin(steer_i)) + v_i*r_i;
    dxdt[2] = u_i*sin(phi_i) + v_i*cos(phi_i);
    dxdt[3] = 1/m*(Fyf*cos(steer_i)+Fyr) - u_i*r_i;
    dxdt[4] = r_i;
    dxdt[5] = 1/Iz*(a*Fyf*cos(steer_i)-b*Fyr);
}

}