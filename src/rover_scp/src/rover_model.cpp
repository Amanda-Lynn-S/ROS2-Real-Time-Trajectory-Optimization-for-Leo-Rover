#include "rover_scp/rover_model.hpp"
#include <unsupported/Eigen/MatrixFunctions> //for .exp() on matrices
#include <cmath>


// Derived constants used in the dynamics equations.
// Matches alpha_beta() in rover_model.py.
static std::pair<double, double> alpha_beta(const RoverParams& p)
{
    double den = p.M * p.c * p.c + p.J;
    return { (p.M * p.c) / den, //alpha
              p.L / (p.R * den) }; //beta
}

// Continuous-time nonlinear dynamics f(x,u).
// Matches f_continuous() in rover_model.py.
State f_continuous(const State& x, const Control& u, const RoverParams& p)
{
    auto [alpha, beta] = alpha_beta(p);
    double th = x(2), v = x(3), om = x(4);
    double TR = u(0), TL = u(1);
    State xdot;
    xdot(0) =  v * std::cos(th);                          //px_dot
    xdot(1) =  v * std::sin(th);                          //py_dot
    xdot(2) =  om;                                        //theta_dot
    xdot(3) =  p.c * om * om + (TR + TL) / (p.R * p.M);   //v_dot
    xdot(4) = -alpha * om * v + beta * (TR - TL);         //omega_dot
    return xdot;
}

// Analytical Jacobian  df/dx  (5×5)
// Replaces  A = jacfwd(f_continuous, argnums=0)(...)  in JAX.
// Non-zero entries:
//   A(0,2) = -v*sin(θ)    A(0,3) = cos(θ)
//   A(1,2) =  v*cos(θ)    A(1,3) = sin(θ)
//   A(2,4) = 1
//   A(3,4) = 2c·ω
//   A(4,3) = -α·ω         A(4,4) = -α·v
MatA df_dx(const State& x, const Control& /*u*/, const RoverParams& p)
{
    auto [alpha, beta] = alpha_beta(p);
    double th = x(2), v = x(3), om = x(4);
    MatA A = MatA::Zero();
    A(0, 2) = -v * std::sin(th);
    A(0, 3) =  std::cos(th);
    A(1, 2) =  v * std::cos(th);
    A(1, 3) =  std::sin(th);
    A(2, 4) =  1.0;
    A(3, 4) =  2.0 * p.c * om;
    A(4, 3) = -alpha * om;
    A(4, 4) = -alpha * v;
    return A;
}

// Analytical Jacobian  df/du  (5×2)
// Replaces  B = jacfwd(f_continuous, argnums=1)(...)  in JAX.
// Non-zero entries:
//   B(3,0) = B(3,1) = 1/(R·M)
//   B(4,0) =  β      B(4,1) = -β
MatB df_du(const State& /*x*/, const Control& /*u*/, const RoverParams& p)
{
    auto [alpha, beta] = alpha_beta(p);
    MatB B = MatB::Zero();
    B(3, 0) =  1.0 / (p.R * p.M);
    B(3, 1) =  1.0 / (p.R * p.M);
    B(4, 0) =  beta;
    B(4, 1) = -beta;
    return B;
}

// Taylor-series linearisation + ZOH discretisation.
// Matches linearize_and_discretize() in rover_model.py.
// Ad = expm(A·dt)
// integral_eAt ≈ Σ_{k=0}^{3} A^k · dt^{k+1} / (k+1)!   (4 terms)
// Bd = integral_eAt · B
// cd = integral_eAt · c_affine
DiscreteSystem linearize_and_discretize(const State& xref,
                                        const Control& uref,
                                        double dt,
                                        const RoverParams& p)
{
    MatA A = df_dx(xref, uref, p);
    MatB B = df_du(xref, uref, p);
    VecC c_af = f_continuous(xref, uref, p) - A * xref - B * uref; //affine error term
    //Matrix exponential via Eigen's Padé approximation:
    MatA Ad = (A * dt).exp();
    //Integral of exp(A*tau) from 0..dt (4-term series):
    Eigen::Matrix<double, 5, 5> integral_eAt = Eigen::Matrix<double, 5, 5>::Zero();
    Eigen::Matrix<double, 5, 5> A_power = Eigen::Matrix<double, 5, 5>::Identity();
    double factorial = 1.0;
    for (int k = 0; k < 4; ++k) {
        factorial *= (k + 1);
        integral_eAt += A_power * std::pow(dt, k + 1) / factorial;
        A_power = A_power * A;
    }
    return { Ad, integral_eAt * B, integral_eAt * c_af };
}
