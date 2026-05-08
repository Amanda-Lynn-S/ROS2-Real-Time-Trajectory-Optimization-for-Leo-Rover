#include "rover_scp/rover.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <limits>

// ============================================================
// Constructor  —  mirrors __init__() in Rover.py
// ============================================================
Rover::Rover(int width_, int height_, int cell_size_,
             const std::vector<int>& start,
             const std::vector<int>& end,
             const Eigen::MatrixXd& sdf_array)
{
    width = width_;
    height = height_;
    cell_size = cell_size_;
    scale = cell_size * pix_to_m; //metres per grid cell
    //SDF in metres (convert from grid-cell units):
    sdf = sdf_array * scale;
    dmin = 0.10; //clearance [m] — rover radius
    //Boundary conditions:
    x0 = State::Zero();
    xf = State::Zero();
    x0(0) = start[0] * scale; //grid gx -> world px
    x0(1) = start[1] * scale; //grid gy -> world py
    xf(0) = end[0] * scale;
    xf(1) = end[1] * scale;
    double dx = xf(0) - x0(0);
    double dy = xf(1) - x0(1);
    double th_goal = std::atan2(dy, dx); //heading straight from start to goal
    x0(2) = th_goal;
    xf(2) = th_goal;
    //State limits:
    const double INF = std::numeric_limits<double>::infinity();
    state_min << 0.0, 0.0, -INF, -0.4, -1.0;
    state_max << width  * cell_size * pix_to_m,
                 height * cell_size * pix_to_m,
                 INF, 0.4, 1.0;
    //Control limits (Leo Rover torque specs):
    u_max <<  5.6,  5.6;
    u_min << -5.6, -5.6;
    //Time discretisation:
    dt = 0.1; //seconds
    double dist = (xf.head<2>() - x0.head<2>()).norm();
    double vmax = state_max(3);
    int    Nmin = static_cast<int>(std::ceil((dist / vmax) / dt)) + 1;
    num_tsteps  = Nmin + 5; //buffer
    //Initial trust region:
    trust_x_init = 10.0;
    trust_u_init = 10.0;
    //Pre-compute SDF gradients:
    compute_gradients();
}

// ============================================================
// Dynamics wrapper
// ============================================================
State Rover::dynamics(const State& x, const Control& u) const
{
    return f_continuous(x, u, RoverParams{});
}

DiscreteSystem Rover::get_discrete(const State& x, const Control& u) const
{
    return linearize_and_discretize(x, u, dt, RoverParams{});
}

// Bilinear interpolation on a 2-D grid:
// grid(row, col) -> grid indexed as (y_idx, x_idx)
// x_m, y_m are world coordinates in meters
// clamps to grid boundary for out-of-range queries (matches fill_value=np.min(sdf) strategy for SDF; for gradients we use 0 fill -> handled by the callers)
double Rover::bilinear(const Eigen::MatrixXd& grid,
                       double x_m, double y_m) const
{
    //Convert world coords -> fractional grid indices:
    double xf_idx = x_m / scale;
    double yf_idx = y_m / scale;
    int x0i = static_cast<int>(std::floor(xf_idx));
    int y0i = static_cast<int>(std::floor(yf_idx));
    int x1i = x0i + 1;
    int y1i = y0i + 1;
    //Clamp indices to valid range:
    int cols = static_cast<int>(grid.cols());
    int rows = static_cast<int>(grid.rows());
    x0i = std::clamp(x0i, 0, cols - 1);
    y0i = std::clamp(y0i, 0, rows - 1);
    x1i = std::clamp(x1i, 0, cols - 1);
    y1i = std::clamp(y1i, 0, rows - 1);
    double tx = xf_idx - std::floor(xf_idx); //fractional part [0,1)
    double ty = yf_idx - std::floor(yf_idx);
    double v00 = grid(y0i, x0i);
    double v10 = grid(y0i, x1i); //+x neighbour
    double v01 = grid(y1i, x0i); //+y neighbour
    double v11 = grid(y1i, x1i);
    return (1-tx)*(1-ty)*v00 + tx*(1-ty)*v10
         + (1-tx)*ty   *v01 + tx*ty   *v11;
}

// Pre-compute gradient arrays via central differences.
// Matches: self.grady, self.gradx = np.gradient(self.sdf, scale, scale)
// np.gradient uses central differences at interior points and one-sided differences at boundaries -> same is done here.
void Rover::compute_gradients()
{
    int R = static_cast<int>(sdf.rows());
    int C = static_cast<int>(sdf.cols());
    grad_x_ = Eigen::MatrixXd::Zero(R, C);
    grad_y_ = Eigen::MatrixXd::Zero(R, C);
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            //x-gradient (along columns):
            double dx;
            if (c == 0)      dx = (sdf(r, 1)   - sdf(r, 0))   / scale;
            else if (c==C-1) dx = (sdf(r, C-1) - sdf(r, C-2)) / scale;
            else             dx = (sdf(r, c+1) - sdf(r, c-1)) / (2.0 * scale);
            grad_x_(r, c) = dx;
            //y-gradient (along rows):
            double dy;
            if (r == 0)      dy = (sdf(1,   c) - sdf(0,   c)) / scale;
            else if (r==R-1) dy = (sdf(R-1, c) - sdf(R-2, c)) / scale;
            else             dy = (sdf(r+1, c) - sdf(r-1, c)) / (2.0 * scale);
            grad_y_(r, c) = dy;
        }
    }
}

// SDF value at world position pos_m = [px, py] in meters.
// Matches Rover.py: self.sdf_interp([pos[::-1]])
// (pos[::-1] reverses [px,py] to [py,px] for row-major lookup)
double Rover::sdf_value(const Eigen::Vector2d& pos_m) const
{
    return bilinear(sdf, pos_m(0), pos_m(1));
}

// SDF gradient at world position pos_m, returns [gx, gy].
// Matches Rover.py: gradx_interp([pos[::-1]]), grady_interp(...)
Eigen::Vector2d Rover::sdf_gradient(const Eigen::Vector2d& pos_m) const
{
    double gx = bilinear(grad_x_, pos_m(0), pos_m(1));
    double gy = bilinear(grad_y_, pos_m(0), pos_m(1));
    return { gx, gy };
}
