// scp.cpp -> Sequential Convex Programming planner -> Replaces SCP.py.
// Algorithm reference: GuSTO (Bonalli et al., ICRA 2019).
// Inner QP is formulated from scratch and solved with OsqpEigen, replacing CVXPY entirely.
// Decision-variable vector z (row-major stacking):
//   [x(0)…x(N-1) | u(0)…u(N-2) | s(0)…s(N-1)]
//   sizes:  N·nx       (N-1)·nu        N
// OSQP problem:  min  ½ zᵀ P z + qᵀ z
//                s.t. l ≤ A z ≤ u


#include "rover_scp/scp.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

using SpMat = Eigen::SparseMatrix<double>;
using Trip  = Eigen::Triplet<double>;

// OSQP's internal infinity is 1e30. Using std::numeric_limits<double>::infinity() can confuse its bound parser, so we cap here.
static constexpr double kInf = 1e30;

// ============================================================
// Constructor
// ============================================================
SCP::SCP(std::shared_ptr<Rover> rover, const rclcpp::Logger& logger)
    : rover_(rover),
      logger_(logger),
      num_tsteps_(rover->num_tsteps),
      dt_(rover->dt),
      trust_x_(rover->trust_x_init),
      trust_u_(rover->trust_u_init)
{
    const int N  = num_tsteps_;
    const int nx = Rover::n_states;    //5
    const int nu = Rover::n_control;   //2
    //Straight-line initialization:
    //np.linspace(x0, xf, N) and 0.01 * np.ones([N-1, nu])
    nom_X_.resize(N, nx);
    for (int t = 0; t < N; ++t) {
        double a      = (N > 1) ? static_cast<double>(t) / (N - 1) : 0.0;
        nom_X_.row(t) = ((1.0 - a) * rover_->x0 + a * rover_->xf).transpose();
    }
    nom_U_ = 0.01 * Eigen::MatrixXd::Ones(N - 1, nu);
}

// ============================================================
// Outer SCP loop -> mirrors SCP.scp() in SCP.py
// ============================================================
void SCP::scp()
{
    double max_state_dev = kInf;
    int    it            = 0;
    Eigen::MatrixXd xprev = nom_X_;
    Eigen::MatrixXd uprev = nom_U_;
    while (max_state_dev >= step_tolerance_ && it < iter_max_) {
        ConvexResult res = convex_program(xprev, uprev);
        //Solver failure -> shrink trust region:
        if (!res.success) {
            trust_x_ *= beta_fail_;
            trust_u_ *= beta_fail_;
            RCLCPP_WARN(logger_,
                "Solve failed — shrink trust to: %.4g / %.4g",
                trust_x_, trust_u_);
            ++it;
            continue;
        }
        //Trust-region violation check:
        //(per-timestep ℓ₂ norms, matching Python's np.linalg.norm(…, axis=1))
        double max_step_x = 0.0, max_step_u = 0.0;
        for (int t = 0; t < num_tsteps_; ++t)
            max_step_x = std::max(max_step_x,
                                  (res.X.row(t) - xprev.row(t)).norm());
        for (int t = 0; t < num_tsteps_ - 1; ++t)
            max_step_u = std::max(max_step_u,
                                  (res.U.row(t) - uprev.row(t)).norm());
        if (max_step_x > trust_x_ || max_step_u > trust_u_) {
            RCLCPP_WARN(logger_,
                "Reject solution — outside trust region: shrink trust");
            trust_x_ *= beta_fail_;
            trust_u_ *= beta_fail_;
            ++it;
            continue;
        }
        //Model-accuracy ratio:
        double rho = compute_rho(res.X, res.U, xprev, uprev);
        if (rho > rho_1_) {
            RCLCPP_WARN(logger_,
                "Reject solution — model inaccurate (rho=%.4g): shrink trust", rho);
            trust_x_ *= beta_fail_;
            trust_u_ *= beta_fail_;
            ++it;
            continue;
        }
        //Accept step:
        max_state_dev = 0.0;
        for (int t = 0; t < num_tsteps_; ++t)
            max_state_dev = std::max(max_state_dev,
                                     (res.X.row(t) - xprev.row(t)).norm());
        RCLCPP_INFO(logger_,
            "Accept solution (rho=%.4g)  iter=%d  max_state_dev=%.6g",
            rho, it + 1, max_state_dev);
        if (rho < rho_0_) {
            trust_x_ *= beta_success_;
            trust_u_ *= beta_success_;
        }
        xprev = res.X;
        uprev = res.U;
        ++it;
    }
    sol_.X = xprev;
    sol_.U = uprev;
    RCLCPP_INFO(logger_, "SCP converged after %d iteration(s).", it);
}

// ============================================================
// convex_program() -> mirrors convex_program() in SCP.py
//
// Builds and solves the following QP each SCP iteration:
//
//   min   ‖U‖²_F  +  w_slack · Σ S[t]
//
//   s.t.  X[0]          = x0                          (IC)
//         X[N-1]         = xf                         (TC)
//         X[t+1]         = Ad·X[t] + Bd·U[t] + cd    (dynamics)
//         state_min ≤ X[t] ≤ state_max                (state bounds)
//         u_min     ≤ U[t] ≤ u_max                    (control bounds)
//         d_nom + ∇d·(X[t,:2]−x_nom) + S[t] ≥ d_min  (collision, softened)
//         ‖X[t,:2] − xprev[t,:2]‖∞ ≤ trust_x         (position trust region)
//         ‖U[t]    − uprev[t]   ‖∞ ≤ trust_u         (control trust region)
//         S[t] ≥ 0                                    (slack non-negativity)
// ============================================================
SCP::ConvexResult SCP::convex_program(
    const Eigen::MatrixXd& xprev,
    const Eigen::MatrixXd& uprev)
{
    const int N  = num_tsteps_;
    const int nx = Rover::n_states;    //5
    const int nu = Rover::n_control;   //2
    const int n_x    = N * nx;           //state variables
    const int n_u    = (N - 1) * nu;     //control variables
    const int n_s    = N;                //slack variables
    const int n_vars = n_x + n_u + n_s;
    // Variable index helpers:
    //x(t, i)  ->  z[ t·nx + i ]
    //u(t, j)  ->  z[ n_x + t·nu + j ]
    //s(t)     ->  z[ n_x + n_u + t ]
    auto xi = [&](int t, int i) -> int { return t * nx + i; };
    auto ui = [&](int t, int j) -> int { return n_x + t * nu + j; };
    auto si = [&](int t)        -> int { return n_x + n_u + t; };
    // Constraint counts:
    const int n_ic  = nx;               // 1. initial condition
    const int n_tc  = nx;               // 2. terminal condition
    const int n_dyn = (N - 1) * nx;    // 3. linearised dynamics
    const int n_sb  = N * nx;          // 4. state bounds
    const int n_cb  = (N - 1) * nu;    // 5. control bounds
    const int n_col = N;               // 6. linearised collision
    const int n_trx = N * 2;           // 7. trust region on positions (indices 0,1)
    const int n_tru = (N - 1) * nu;   // 8. trust region on controls
    const int n_sl  = N;               // 9. slack non-negativity
    const int n_con = n_ic + n_tc + n_dyn + n_sb + n_cb
                    + n_col + n_trx + n_tru + n_sl;
    // Pre-compute ZOH-discretised systems for each interval:
    std::vector<DiscreteSystem> disc(N - 1);
    for (int t = 0; t < N - 1; ++t) {
        State   xt = xprev.row(t).transpose();
        Control ut = uprev.row(t).transpose();
        disc[t]    = rover_->get_discrete(xt, ut);
    }
    // Build sparse constraint matrix A and bound vectors:
    std::vector<Trip> trips_A;
    trips_A.reserve(n_con * 12); //generous upper bound on non-zeros
    Eigen::VectorXd lb(n_con), ub(n_con);
    lb.setConstant(-kInf);
    ub.setConstant( kInf);
    int row = 0;
    // 1. Initial condition: X[0] == x0
    for (int i = 0; i < nx; ++i) {
        trips_A.emplace_back(row, xi(0, i), 1.0);
        lb(row) = ub(row) = rover_->x0(i);
        ++row;
    }
    // 2. Terminal condition: X[N-1] == xf 
    for (int i = 0; i < nx; ++i) {
        trips_A.emplace_back(row, xi(N - 1, i), 1.0);
        lb(row) = ub(row) = rover_->xf(i);
        ++row;
    }
    // 3. Linearised dynamics: X[t+1] − Ad·X[t] − Bd·U[t] == cd 
    for (int t = 0; t < N - 1; ++t) {
        const auto& d = disc[t];
        for (int i = 0; i < nx; ++i) {
            //+1 on X[t+1, i]
            trips_A.emplace_back(row, xi(t + 1, i), 1.0);
            //−Ad[i,j] on X[t, j]
            for (int j = 0; j < nx; ++j)
                if (d.Ad(i, j) != 0.0)
                    trips_A.emplace_back(row, xi(t, j), -d.Ad(i, j));
            //−Bd[i,j] on U[t, j]
            for (int j = 0; j < nu; ++j)
                if (d.Bd(i, j) != 0.0)
                    trips_A.emplace_back(row, ui(t, j), -d.Bd(i, j));
            lb(row) = ub(row) = d.cd(i);
            ++row;
        }
    }
    // 4. State bounds: state_min ≤ X[t] ≤ state_max (clamp ±∞ from rover to OSQP's numerical infinity)
    for (int t = 0; t < N; ++t) {
        for (int i = 0; i < nx; ++i) {
            trips_A.emplace_back(row, xi(t, i), 1.0);
            lb(row) = std::max(rover_->state_min(i), -kInf);
            ub(row) = std::min(rover_->state_max(i),  kInf);
            ++row;
        }
    }
    // 5. Control bounds: u_min ≤ U[t] ≤ u_max
    for (int t = 0; t < N - 1; ++t) {
        for (int j = 0; j < nu; ++j) {
            trips_A.emplace_back(row, ui(t, j), 1.0);
            lb(row) = rover_->u_min(j);
            ub(row) = rover_->u_max(j);
            ++row;
        }
    }
    // 6. Linearised collision avoidance constraint
    // d_nom + ∇d · (X[t,:2] − x_nom) + S[t] ≥ dmin
    // ⟹  ∇dᵀ X[t,:2] + S[t]  ≥  dmin − d_nom + ∇dᵀ x_nom
    for (int t = 0; t < N; ++t) {
        Eigen::Vector2d x_nom = xprev.row(t).head<2>().transpose();
        double          d_nom = rover_->sdf_value(x_nom);
        Eigen::Vector2d n_nom = rover_->sdf_gradient(x_nom);
        trips_A.emplace_back(row, xi(t, 0), n_nom(0));
        trips_A.emplace_back(row, xi(t, 1), n_nom(1));
        trips_A.emplace_back(row, si(t),    1.0);
        lb(row) = rover_->dmin - d_nom + n_nom.dot(x_nom);
        ub(row) = kInf;
        ++row;
    }
    // 7. Trust region on positions (element-wise ∞-norm)
    // xprev[t,i] − trust_x ≤ X[t,i] ≤ xprev[t,i] + trust_x for i ∈ {0,1}
    for (int t = 0; t < N; ++t) {
        for (int i = 0; i < 2; ++i) {
            trips_A.emplace_back(row, xi(t, i), 1.0);
            lb(row) = xprev(t, i) - trust_x_;
            ub(row) = xprev(t, i) + trust_x_;
            ++row;
        }
    }
    // 8. Trust region on controls (element-wise ∞-norm)
    // uprev[t,j] − trust_u ≤ U[t,j] ≤ uprev[t,j] + trust_u
    for (int t = 0; t < N - 1; ++t) {
        for (int j = 0; j < nu; ++j) {
            trips_A.emplace_back(row, ui(t, j), 1.0);
            lb(row) = uprev(t, j) - trust_u_;
            ub(row) = uprev(t, j) + trust_u_;
            ++row;
        }
    }
    // 9. Slack non-negativity: S[t] ≥ 0 
    for (int t = 0; t < N; ++t) {
        trips_A.emplace_back(row, si(t), 1.0);
        lb(row) = 0.0;
        ub(row) = kInf;
        ++row;
    }
    assert(row == n_con && "Constraint row count mismatch — logic error in convex_program()");
    SpMat A_sp(n_con, n_vars);
    A_sp.setFromTriplets(trips_A.begin(), trips_A.end());
    // Cost matrix P (upper triangular, diagonal only) and linear cost vector q:
    // min  ‖U‖²_F  +  w_slack · Σ S[t]
    //    = ½ zᵀ P z + qᵀ z
    //   ⟹ P[ui(t,j), ui(t,j)] = 2  (so ½·2·u² = u²)
    //      q[si(t)]             = w_slack
    const double w_slack = 1e4;
    std::vector<Trip> trips_P;
    trips_P.reserve(n_u);
    for (int t = 0; t < N - 1; ++t)
        for (int j = 0; j < nu; ++j)
            trips_P.emplace_back(ui(t, j), ui(t, j), 2.0);
    SpMat P_sp(n_vars, n_vars);
    P_sp.setFromTriplets(trips_P.begin(), trips_P.end());
    Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(n_vars);
    for (int t = 0; t < N; ++t)
        q_vec(si(t)) = w_slack;
    // Set up and solve with OsqpEigen:
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.settings()->setMaxIteration(50000);
    solver.settings()->setAbsoluteTolerance(1e-3);
    solver.settings()->setRelativeTolerance(1e-3);
    solver.settings()->setPolish(true);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_con);
    if (!solver.data()->setHessianMatrix(P_sp))           return {false, {}, {}};
    if (!solver.data()->setGradient(q_vec))               return {false, {}, {}};
    if (!solver.data()->setLinearConstraintsMatrix(A_sp)) return {false, {}, {}};
    if (!solver.data()->setLowerBound(lb))                return {false, {}, {}};
    if (!solver.data()->setUpperBound(ub))                return {false, {}, {}};
    if (!solver.initSolver()) {
        RCLCPP_WARN(logger_, "OsqpEigen: solver initialisation failed.");
        return {false, {}, {}};
    }
    solver.solve();
    auto status = solver.getStatus();
    bool solved     = (status == OsqpEigen::Status::Solved);
    bool inaccurate = (status == OsqpEigen::Status::SolvedInaccurate);
    bool max_iter   = (status == OsqpEigen::Status::MaxIterReached);
    if (!solved && !inaccurate && !max_iter) {
        RCLCPP_WARN(logger_, "OSQP: infeasible or other failure (status=%d).",
                    static_cast<int>(status));
        return {false, {}, {}};
    }
    if (max_iter)
        RCLCPP_WARN(logger_,
            "Warning: OSQP reached max iterations — accepting solution.");
    Eigen::VectorXd z = solver.getSolution();
    // Extract X and U from flat solution vector:
    Eigen::MatrixXd X_sol(N,     nx);
    Eigen::MatrixXd U_sol(N - 1, nu);
    for (int t = 0; t < N;     ++t)
        for (int i = 0; i < nx; ++i)
            X_sol(t, i) = z(xi(t, i));
    for (int t = 0; t < N - 1; ++t)
        for (int j = 0; j < nu; ++j)
            U_sol(t, j) = z(ui(t, j));
    // Log max slack (mirrors Python: print("max slack:", np.max(S.value))):
    double max_slack = 0.0;
    for (int t = 0; t < N; ++t)
        max_slack = std::max(max_slack, z(si(t)));
    RCLCPP_INFO(logger_, "max slack: %.4g", max_slack);
    return {true, std::move(X_sol), std::move(U_sol)};
}

// compute_rho() -> mirrors compute_rho() in SCP.py:
// ρ = Σ ‖x_true_next − x_lin_next‖ / (Σ ‖x_lin_next‖ + ε)
// Linearisation is around (xprev, uprev).
// Both x_lin_next and x_true_next are evaluated at (x_sol, u_sol).
// "True" step uses forward Euler on continuous-time dynamics, matching the Python implementation exactly.
double SCP::compute_rho(const Eigen::MatrixXd& x_sol,
                        const Eigen::MatrixXd& u_sol,
                        const Eigen::MatrixXd& xprev,
                        const Eigen::MatrixXd& uprev)
{
    double num = 0.0, den = 0.0;
    for (int t = 0; t < num_tsteps_ - 1; ++t) {
        State   xt_prev = xprev.row(t).transpose();
        Control ut_prev = uprev.row(t).transpose();
        State   xs      = x_sol.row(t).transpose();
        Control us      = u_sol.row(t).transpose();
        //Linearised prediction (around xprev/uprev, at x_sol/u_sol):
        auto  d          = rover_->get_discrete(xt_prev, ut_prev);
        State x_lin_next = d.Ad * xs + d.Bd * us + d.cd;
        // "True" forward-Euler step on nonlinear dynamics:
        State f_val       = rover_->dynamics(xs, us);
        State x_true_next = xs + dt_ * f_val;
        num += (x_true_next - x_lin_next).norm();
        den += x_lin_next.norm();
    }
    return num / (den + 1e-9);
}
