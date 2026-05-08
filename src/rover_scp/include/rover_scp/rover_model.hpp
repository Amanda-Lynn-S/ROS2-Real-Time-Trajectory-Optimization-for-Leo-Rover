#pragma once
#include <Eigen/Dense>


// Rover Dynamics Model:
// State:   x = [px, py, theta, v, omega]   (5 states)
// Control: u = [TR, TL]                    (2 controls, wheel torques)
// Replaces rover_model.py — Jacobians are derived analytically here instead of using JAX autodiff. 
// This is the primary speed-up: no Python / XLA overhead per SCP iteration.


// Convenience type aliases so the rest of the code stays readable:
using State   = Eigen::Matrix<double, 5, 1>;
using Control = Eigen::Matrix<double, 2, 1>;
using MatA    = Eigen::Matrix<double, 5, 5>;
using MatB    = Eigen::Matrix<double, 5, 2>;
using VecC    = Eigen::Matrix<double, 5, 1>;

// Physical rover parameters (Leo Rover defaults):
struct RoverParams {
    double M = 7.0;    //mass [kg]
    double J = 0.22;   //rotational inertia [kg·m²]
    double R = 0.129;  //wheel radius [m]
    double L = 0.177;  //half wheel-track [m]
    double c = 0.0;    //CoM offset from mid-axle [m]
};

// ZOH-discretised linearised system around one trajectory point:
struct DiscreteSystem {
    MatA Ad;   //discrete state transition matrix  (5×5)
    MatB Bd;   //discrete control matrix           (5×2)
    VecC cd;   //discrete affine term              (5×1)
};

// ---- Continuous-time nonlinear dynamics ----
State f_continuous(const State& x, const Control& u,
                   const RoverParams& p = RoverParams{});

// ---- Analytical Jacobians (replaces jacfwd in JAX) ----
MatA df_dx(const State& x, const Control& u,
           const RoverParams& p = RoverParams{});

MatB df_du(const State& x, const Control& u,
           const RoverParams& p = RoverParams{});

// ---- Taylor-series linearisation + ZOH discretisation ----
// Equivalent to linearize_and_discretize() in rover_model.py.
DiscreteSystem linearize_and_discretize(const State& xref,
                                        const Control& uref,
                                        double dt,
                                        const RoverParams& p = RoverParams{});
