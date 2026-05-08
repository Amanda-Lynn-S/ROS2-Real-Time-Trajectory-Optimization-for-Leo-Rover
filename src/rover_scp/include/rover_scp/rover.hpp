#pragma once
#include "rover_scp/rover_model.hpp"
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <limits>


// Rover class -> replaces Rover.py:
// Holds all constraint parameters, the SDF grid (in metres), and provides interpolation helpers used by SCP.
// Constructed by rover_pp_node once it has received both /map_meta and /sdf_grid from map_publisher_node.


class Rover
{
public:
    //Fixed dimensions:
    static constexpr int n_states  = 5;
    static constexpr int n_control = 2;
    //Map geometry:
    int    width;       //grid columns
    int    height;      //grid rows
    int    cell_size;   //pixels per cell (same meaning as in Rover.py)
    double scale;       //metres per cell = cell_size * pix_to_m
    //Boundary conditions:
    State x0;           //initial state [px,py,θ,v,ω]
    State xf;           //terminal state
    //State box constraints:
    State state_min;    //lower bounds
    State state_max;    //upper bounds
    //Control box constraints:
    Control u_max;
    Control u_min;
    //Collision avoidance:
    Eigen::MatrixXd sdf;   //H×W SDF in metres (negative inside obstacle)
    double dmin;           //clearance [m]
    //Time discretisation:
    double dt;
    int    num_tsteps;
    //Initial trust-region radii:
    double trust_x_init;
    double trust_u_init;

    // ---- Constructor ----
    // meta_int  : {"width", "height", "cell_size"}
    // start/end : grid-cell coordinates  [gx, gy]
    // sdf_array : H×W float matrix (grid-cell units, from /sdf_grid)
    Rover(int width, int height, int cell_size,
          const std::vector<int>& start,
          const std::vector<int>& end,
          const Eigen::MatrixXd& sdf_array);

    // ---- Dynamics wrappers (used by SCP::compute_rho) ----
    State dynamics(const State& x, const Control& u) const;
    DiscreteSystem get_discrete(const State& x, const Control& u) const;

    // ---- SDF interpolation (bilinear, matches scipy RegularGridInterpolator) ----
    double        sdf_value   (const Eigen::Vector2d& pos_m) const;
    Eigen::Vector2d sdf_gradient(const Eigen::Vector2d& pos_m) const;

private:
    static constexpr double pix_to_m = 0.01; //must match map_publisher_node.py
    Eigen::MatrixXd grad_x_; //d(sdf)/dx at each grid cell [m/m]
    Eigen::MatrixXd grad_y_; //d(sdf)/dy
    double bilinear(const Eigen::MatrixXd& grid,
                    double x_m, double y_m) const;
    void   compute_gradients();
};
