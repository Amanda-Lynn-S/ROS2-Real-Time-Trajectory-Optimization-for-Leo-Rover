#pragma once
#include "rover_scp/rover.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <rclcpp/logger.hpp>
#include <memory>


// SCP -> Sequential Convex Programming planner -> Replaces SCP.py.
// Each call to scp() runs the outer algorithm loop.
// Each inner call to convex_program() formulates a QP and solves it directly with OSQP (via OsqpEigen), eliminating all CVXPY / Python overhead.


struct SCPResult {
    Eigen::MatrixXd X;   //N × 5 state trajectory
    Eigen::MatrixXd U;   //(N-1) × 2 control trajectory
};

class SCP
{
public:
    SCP(std::shared_ptr<Rover> rover, const rclcpp::Logger& logger);
    //Run the outer SCP loop. Results stored in sol_:
    void scp();
    //Access solution after convergence:
    const SCPResult& solution() const { return sol_; }
private:
    std::shared_ptr<Rover> rover_;
    rclcpp::Logger         logger_;
    int    num_tsteps_;
    double dt_;
    //Initial nominal trajectory (straight-line guess):
    Eigen::MatrixXd nom_X_;   //N × 5
    Eigen::MatrixXd nom_U_;   //(N-1) × 2
    //Trust region (mutable during iteration):
    double trust_x_;
    double trust_u_;
    //Algorithm parameters (same defaults as SCP.py):
    double beta_fail_      = 0.8;
    double beta_success_   = 1.0;
    double rho_0_          = 0.5;
    double rho_1_          = 1e-1;
    double step_tolerance_ = 0.001;
    int    iter_max_       = 20;
    SCPResult sol_;
    //Inner QP solve:
    struct ConvexResult {
        bool           success;
        Eigen::MatrixXd X;
        Eigen::MatrixXd U;
    };
    ConvexResult convex_program(const Eigen::MatrixXd& xprev,
                                const Eigen::MatrixXd& uprev);
    //Model accuracy ratio (same as compute_rho in SCP.py):
    double compute_rho(const Eigen::MatrixXd& x_sol,
                       const Eigen::MatrixXd& u_sol,
                       const Eigen::MatrixXd& xprev,
                       const Eigen::MatrixXd& uprev);
};
