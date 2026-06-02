#pragma once
#include <memory>
#include <string>
#include <vector>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include "ramppi_controller/mppi.hpp"

namespace ramppi_controller {

class RAMPPIController : public nav2_core::Controller {
public:
    RAMPPIController() = default;
    ~RAMPPIController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override {}
    void activate() override {}
    void deactivate() override {}

    void setPlan(const nav_msgs::msg::Path& path) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker* goal_checker) override;

    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string name_;
    nav_msgs::msg::Path plan_;
    rclcpp::Logger logger_{rclcpp::get_logger("RAMPPIController")};

    double speed_limit_{1.0};
    double prev_target_angle_{0.0};
    bool has_prev_angle_{false};
    uint32_t ramppi_seed_{42};
    float prev_best_vx_{0.0f};
    float prev_best_vy_{0.0f};
    bool frames_logged_{false};

    std::vector<ObstacleState> LoadObstacles(
        nav2_costmap_2d::Costmap2D* costmap) const;

    static constexpr double KP_ANG = 2.0;
    static constexpr double KP_LIN = 0.9;
    static constexpr double VX_MAX = 0.5;
    static constexpr double WZ_MAX = 1.6;
    static constexpr double LOOKAHEAD = 0.80;
    static constexpr double ANGLE_SMOOTH_ALPHA = 0.7;
    static constexpr float  DT_STEP = 0.1f;  // seconds per MPPI step
    static constexpr float  OBS_RADIUS_M = 0.15f;  // cylinder radius
    static constexpr float  OBS_HARD_MARGIN = 0.09f;
    static constexpr float  OBS_SOFT_M = 0.70f;
};

}
