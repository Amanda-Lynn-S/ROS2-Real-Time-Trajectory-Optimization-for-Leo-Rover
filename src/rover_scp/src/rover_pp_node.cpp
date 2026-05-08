// ============================================================
// rover_pp_node.cpp
// ROS2 C++ node that replaces rover_pp_node.py.
//
// Subscribes to:  /map_meta   (std_msgs/String  — JSON)
//                 /sdf_grid   (std_msgs/Float32MultiArray)
// Publishes  to:  /scp_trajectory_states
//                 /scp_trajectory_controls
//                 /cmd_vel    (geometry_msgs/Twist)
//
// Flow (identical to Python version):
//   1. Wait for both /map_meta and /sdf_grid.
//   2. Instantiate Rover + SCP, run optimiser in a background thread.
//   3. Publish full trajectory once, then stream /cmd_vel at dt rate.
//
// Note: a lightweight "watchdog" timer (10 ms) runs on
// the executor thread, detects when the background SCP thread is
// done (via traj_ready_ atomic flag), and creates the cmd_vel timer from there.
// ============================================================


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nlohmann/json.hpp>
#include <Eigen/Dense>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <chrono>

#include "rover_scp/rover.hpp"
#include "rover_scp/scp.hpp"

using json = nlohmann::json;


class RoverPPNode : public rclcpp::Node
{
public:
    RoverPPNode() : Node("rover_pp_node"), scp_started_(false),
                    traj_ready_(false), traj_index_(0), dt_step_(0.1)
    {
        // ---- Subscribers ----
        meta_sub_ = create_subscription<std_msgs::msg::String>(
            "/map_meta", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) { meta_callback(msg); });
        sdf_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/sdf_grid", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { sdf_callback(msg); });
        // ---- Publishers ----
        states_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/scp_trajectory_states", 10);
        ctrl_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "/scp_trajectory_controls", 10);
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        // ---- Watchdog timer (executor thread) ----
        // Fires every 10 ms; once SCP is done it publishes the trajectory and creates the cmd_vel timer — both safely on the executor thread:
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { watchdog_callback(); });
        RCLCPP_INFO(get_logger(),
                    "rover_pp_node (C++) ready — waiting for /map_meta and /sdf_grid.");
    }
    ~RoverPPNode() override
    {
        // Join the SCP thread to avoid destroying 'this' while it still runs:
        if (scp_thread_.joinable())
            scp_thread_.join();
    }

private:
    // ---- Subscriptions / Publications ----
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              meta_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   sdf_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      states_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      ctrl_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;
    rclcpp::TimerBase::SharedPtr                                        watchdog_timer_;
    rclcpp::TimerBase::SharedPtr                                        cmd_timer_;
    // ---- Incoming map data (written in callbacks, read in try_run_scp) ----
    json            meta_json_;
    bool            meta_received_ = false;
    bool            sdf_received_  = false;
    Eigen::MatrixXd sdf_matrix_;
    // ---- Background SCP thread ----
    std::atomic<bool> scp_started_; //prevents double-launch
    std::thread       scp_thread_;
    // ---- Trajectory ready flag + result (written by scp_thread, read by watchdog) ----
    std::atomic<bool> traj_ready_;
    std::mutex        traj_mutex_;
    Eigen::MatrixXd   X_traj_;        //N × 5  (protected by traj_mutex_)
    Eigen::MatrixXd   U_traj_;        //(N-1) × 2
    // ---- Trajectory playback (only accessed from executor thread) ----
    int    traj_index_;
    double dt_step_;

    // ================================================================
    // Subscriber callbacks
    // ================================================================

    void meta_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (scp_started_) return;
        meta_json_     = json::parse(msg->data);
        meta_received_ = true;
        RCLCPP_INFO(get_logger(), "Received /map_meta.");
        try_run_scp();
    }

    void sdf_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (scp_started_) return;
        int rows = msg->layout.dim[0].size;
        int cols = msg->layout.dim[1].size;
        sdf_matrix_ = Eigen::MatrixXd(rows, cols);
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c)
                sdf_matrix_(r, c) = static_cast<double>(msg->data[r * cols + c]);
        sdf_received_ = true;
        RCLCPP_INFO(get_logger(), "Received /sdf_grid (%d×%d).", rows, cols);
        try_run_scp();
    }

    // ================================================================
    // Planning -> launch background thread once both topics are received
    // ================================================================

    void try_run_scp()
    {
        if (!meta_received_ || !sdf_received_) return;
        bool expected = false;
        if (!scp_started_.compare_exchange_strong(expected, true)) return;
        scp_thread_ = std::thread([this]() { run_scp(); });
    }

    void run_scp()
    {
        // ---- Parse /map_meta JSON ----
        int  width     = meta_json_["width"];
        int  height    = meta_json_["height"];
        int  cell_size = meta_json_["cell_size"];
        std::vector<int> start = { meta_json_["start"][0], meta_json_["start"][1] };
        std::vector<int> end   = { meta_json_["end"][0],   meta_json_["end"][1]   };
        // ---- Build Rover and SCP objects ----
        auto rover = std::make_shared<Rover>(width, height, cell_size,
                                             start, end, sdf_matrix_);
        auto scp_planner = std::make_shared<SCP>(rover, get_logger());
        // ---- Run optimizer ----
        auto t0 = std::chrono::steady_clock::now();
        scp_planner->scp();
        auto t1 = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t1 - t0).count();
        RCLCPP_INFO(get_logger(), "SCP completed in %.3f s.", elapsed);
        // ---- Store dt and trajectory under mutex, then signal watchdog ----
        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            dt_step_ = rover->dt;
            X_traj_  = scp_planner->solution().X;
            U_traj_  = scp_planner->solution().U;
        }
        traj_ready_.store(true, std::memory_order_release);
    }

    // ================================================================
    // Watchdog callback (runs on executor thread every 10 ms)
    // Detects SCP completion and safely creates the cmd_vel timer.
    // ================================================================

    void watchdog_callback()
    {
        if (!traj_ready_.load(std::memory_order_acquire)) return;
        // Cancel watchdog (we only need to act once):
        watchdog_timer_->cancel();
        // Read trajectory under mutex, then publish:
        Eigen::MatrixXd X, U;
        {
            std::lock_guard<std::mutex> lk(traj_mutex_);
            X = X_traj_;
            U = U_traj_;
        }
        publish_trajectory(X, U);
        // Create cmd_vel timer:
        traj_index_ = 0;
        auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt_step_));
        cmd_timer_ = create_wall_timer(period,
            [this]() { publish_cmd_vel(); });
    }

    // ================================================================
    // Publishing helpers
    // ================================================================

    std_msgs::msg::Float32MultiArray make_float32_msg(
        const Eigen::MatrixXd& arr,
        const std::string& dim0_label,
        const std::string& dim1_label)
    {
        std_msgs::msg::Float32MultiArray msg;
        std_msgs::msg::MultiArrayDimension d0, d1;
        d0.label  = dim0_label;
        d0.size   = static_cast<uint32_t>(arr.rows());
        d0.stride = static_cast<uint32_t>(arr.rows() * arr.cols());
        d1.label  = dim1_label;
        d1.size   = static_cast<uint32_t>(arr.cols());
        d1.stride = static_cast<uint32_t>(arr.cols());
        msg.layout.dim        = { d0, d1 };
        msg.layout.data_offset = 0;
        msg.data.reserve(static_cast<std::size_t>(arr.size()));
        for (int r = 0; r < arr.rows(); ++r)
            for (int c = 0; c < arr.cols(); ++c)
                msg.data.push_back(static_cast<float>(arr(r, c)));
        return msg;
    }

    void publish_trajectory(const Eigen::MatrixXd& X, const Eigen::MatrixXd& U)
    {
        states_pub_->publish(make_float32_msg(X, "timesteps", "states"));
        ctrl_pub_->publish(make_float32_msg(U, "timesteps", "controls"));
        RCLCPP_INFO(get_logger(),
                    "Published trajectory: states %dx%d, controls %dx%d.",
                    static_cast<int>(X.rows()), static_cast<int>(X.cols()),
                    static_cast<int>(U.rows()), static_cast<int>(U.cols()));
    }

    void publish_cmd_vel()
    {
        // X_traj_ is stable by this point (SCP thread is done) / no lock needed:
        if (traj_index_ >= static_cast<int>(X_traj_.rows())) {
            cmd_timer_->cancel();
            RCLCPP_INFO(get_logger(), "Trajectory playback complete.");
            return;
        }
        auto state = X_traj_.row(traj_index_);
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = state(3);   //v [m/s]
        twist.angular.z = state(4);   //omega [rad/s]
        cmd_pub_->publish(twist);
        ++traj_index_;
    }
};

// ================================================================
// main
// ================================================================
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPPNode>());
    rclcpp::shutdown();
    return 0;
}
