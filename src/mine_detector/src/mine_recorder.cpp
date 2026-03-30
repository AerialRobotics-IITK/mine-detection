#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"

#include <gpiod.hpp> // The C++ GPIO library

// =================================================================
// 1. LoggedTrajectory Class: Handles discrete path storage & interpolation
// =================================================================
class LoggedTrajectory {
public:
    struct Entry {
        double elapsed_time; // Time relative to mission start
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
    };

private:
    std::vector<Entry> data_;

public:
    void append(double t, const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
        data_.push_back({t, p, v});
    }

    // Linearly interpolate position at time t
    Eigen::Vector3d getPos(double t) const {
        if (data_.empty()) return Eigen::Vector3d::Zero();
        if (t <= data_.front().elapsed_time) return data_.front().pos;
        if (t >= data_.back().elapsed_time) return data_.back().pos;

        auto it = std::lower_bound(data_.begin(), data_.end(), t, 
            [](const Entry& e, double t_val) { return e.elapsed_time < t_val; });

        auto prev = std::prev(it);
        double ratio = (t - prev->elapsed_time) / (it->elapsed_time - prev->elapsed_time);
        return prev->pos + ratio * (it->pos - prev->pos);
    }

    // Linearly interpolate velocity at time t
    Eigen::Vector3d getVel(double t) const {
        if (data_.empty()) return Eigen::Vector3d::Zero();
        if (t <= data_.front().elapsed_time) return data_.front().vel;
        if (t >= data_.back().elapsed_time) return data_.back().vel;

        auto it = std::lower_bound(data_.begin(), data_.end(), t, 
            [](const Entry& e, double t_val) { return e.elapsed_time < t_val; });

        auto prev = std::prev(it);
        double ratio = (t - prev->elapsed_time) / (it->elapsed_time - prev->elapsed_time);
        return prev->vel + ratio * (it->vel - prev->vel);
    }

    bool empty() const { return data_.empty(); }
    void clear() { data_.clear(); }
};

// =================================================================
// 2. MineLocalizationNode: The main ROS 2 Logic
// =================================================================
class MineLocalizationNode : public rclcpp::Node {
public:
    MineLocalizationNode() : Node("mine_localization_node") {
        // Parameters
        this->declare_parameter<double>("velocity_threshold", 0.15); // m/s
        this->declare_parameter<double>("stability_window", 0.1);    // seconds
        
        vel_threshold_ = this->get_parameter("velocity_threshold").as_double();
        stability_window_ = this->get_parameter("stability_window").as_double();

        // Subscriptions
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "uav3/local_position/pose", 10, std::bind(&MineLocalizationNode::pose_cb, this, std::placeholders::_1));
        
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "uav3/local_position/velocity_local", 10, std::bind(&MineLocalizationNode::vel_cb, this, std::placeholders::_1));

        // Sync with port.cpp: Start Time
        start_time_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/detector_start_time", 10, std::bind(&MineLocalizationNode::start_time_cb, this, std::placeholders::_1));

        // THE TRIGGER: receives 'pt.t' from port.cpp
        timestamp_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/timestamp_pipeline", 10, std::bind(&MineLocalizationNode::timestamp_cb, this, std::placeholders::_1));

        // Publisher
        mine_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/detected_mine_pos", 10);

        // Timer to record trajectory history at 50Hz (every 20ms)
        record_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&MineLocalizationNode::record_path_step, this));


        // --- GPIO Initialization ---
        try {
            // "0" is usually the GPIO chip for RPi 4/5
            auto chip = gpiod::chip("0"); 
            buzzer_line_ = chip.get_line(18); // BCM Pin 18

            buzzer_line_.request({"mine_buzzer", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);

            RCLCPP_INFO(this->get_logger(), "GPIO Buzzer initialized on BCM 18");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "Mine Localization Node Started. Waiting for start time sync...");
    }

private:
    // Callback: Reference Start Time from port.cpp
    void start_time_cb(const std_msgs::msg::Float64::SharedPtr msg) {
        if (!has_start_time_) {
            shared_start_time_ = msg->data;
            has_start_time_ = true;
            RCLCPP_INFO(this->get_logger(), "Clock Synchronized with port.cpp. Mission Start: %.2f", shared_start_time_);
        }
    }

    // Callback: Pose
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        
        latest_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        has_odom_ = true;
    }

    // Callback: Velocity
    void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        latest_vel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    }

    // Timer Loop: Save the current state into history
    void record_path_step() {
        if (!has_odom_ || !has_start_time_) return;

        // Calculate time relative to the shared mission start
        double elapsed_t = this->now().seconds() - shared_start_time_;
        history_.append(elapsed_t, latest_pos_, latest_vel_);
    }

    // Callback: The Core Logic triggered by port.cpp
    void timestamp_cb(const std_msgs::msg::Float64::SharedPtr msg) {
        double T_trigger = msg->data; // This is 'pt.t' (elapsed time)

        if (T_trigger <= 0.0) return; // Skip non-triggered signals (-1.0)
        if (!has_start_time_) {
            RCLCPP_WARN(this->get_logger(), "Trigger received but start time not synced!");
            return;
        }
        if (history_.empty()) return;

        // --- YOUR CORRECT LOGIC ---
        // We check the state at T - dt and T + dt to ensure no transition was happening
        double vel_norm_before = history_.getVel(T_trigger - stability_window_).norm();
        double vel_norm_after  = history_.getVel(T_trigger + stability_window_).norm();

        bool is_rest_before = (vel_norm_before < vel_threshold_);
        bool is_rest_after  = (vel_norm_after < vel_threshold_);

        // If State Before == State After, the drone is in a stable state (Steady Rest or Steady Move)
        if (is_rest_before == is_rest_after) {
            // NOTE DOWN THE POSITION
            Eigen::Vector3d mine_coords = history_.getPos(T_trigger);

            geometry_msgs::msg::Point out_msg;
            out_msg.x = mine_coords.x();
            out_msg.y = mine_coords.y();
            out_msg.z = mine_coords.z();
            mine_pub_->publish(out_msg);

            RCLCPP_INFO(this->get_logger(), "MINE LOGGED at T=%.2f. Pos: [%.2f, %.2f, %.2f] | State: %s", 
                        T_trigger, out_msg.x, out_msg.y, out_msg.z, 
                        is_rest_before ? "STABLE REST" : "STABLE MOVEMENT");

            trigger_buzzer(500); // Beep for 500ms when a mine is confirmed
        } 
        else {
            // IGNORE: State was changing (Drone was starting to move or starting to stop)
            RCLCPP_WARN(this->get_logger(), "Trigger Ignored: Drone was changing state (Transition) at T=%.2f", T_trigger);
        }
    }

     // Helper function to trigger buzzer without blocking ROS
    void trigger_buzzer(int milliseconds) {
        // Run buzzer in a separate thread so it doesn't stop the path recording
        std::thread([this, milliseconds]() {
            try {
                buzzer_line_.set_value(1); // ON
                std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
                buzzer_line_.set_value(0); // OFF
            } catch (...) {}
        }).detach();
    }

    // Variables
    LoggedTrajectory history_;
    Eigen::Vector3d latest_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_vel_ = Eigen::Vector3d::Zero();
    
    double shared_start_time_ = 0.0;
    bool has_start_time_ = false;
    bool has_odom_ = false;

    // Params
    double vel_threshold_;
    double stability_window_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr start_time_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr timestamp_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr mine_pub_;
    rclcpp::TimerBase::SharedPtr record_timer_;

    gpiod::line buzzer_line_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MineLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
