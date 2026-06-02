#include "ramppi_controller/ramppi_controller.hpp"
#include "ramppi_controller/field.hpp"
#include "ramppi_controller/ramppi.hpp"
#include <nav2_core/controller_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <fstream>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(ramppi_controller::RAMPPIController, nav2_core::Controller)

namespace ramppi_controller {

void RAMPPIController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    RCLCPP_INFO(logger_, "RAMPPIController configured");
}

void RAMPPIController::setPlan(const nav_msgs::msg::Path& path) {
    plan_ = path;
    has_prev_angle_ = false;
    prev_best_vx_ = 0.0f;
    prev_best_vy_ = 0.0f;

    const std::string target_frame = costmap_ros_ ? costmap_ros_->getGlobalFrameID()
                                                  : std::string("odom");
    if (!plan_.poses.empty() && plan_.header.frame_id != target_frame && tf_) {
        nav_msgs::msg::Path transformed;
        transformed.header.stamp = plan_.header.stamp;
        transformed.header.frame_id = target_frame;
        transformed.poses.reserve(plan_.poses.size());
        try {
            geometry_msgs::msg::TransformStamped tf_msg = tf_->lookupTransform(
                target_frame, plan_.header.frame_id, tf2::TimePointZero);
            for (const auto& p : plan_.poses) {
                geometry_msgs::msg::PoseStamped in = p;
                if (in.header.frame_id.empty()) in.header.frame_id = plan_.header.frame_id;
                geometry_msgs::msg::PoseStamped out;
                tf2::doTransform(in, out, tf_msg);
                out.header.frame_id = target_frame;
                transformed.poses.push_back(out);
            }
            plan_ = transformed;
            RCLCPP_INFO(logger_,
                "[RAMPPI_TF] plan transformed: %s -> %s, n=%zu, "
                "first=(%.2f,%.2f) last=(%.2f,%.2f)",
                path.header.frame_id.c_str(), target_frame.c_str(),
                plan_.poses.size(),
                plan_.poses.front().pose.position.x, plan_.poses.front().pose.position.y,
                plan_.poses.back().pose.position.x, plan_.poses.back().pose.position.y);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(logger_,
                "[RAMPPI_TF] failed to transform plan %s -> %s: %s. Using raw plan.",
                plan_.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        }
    }
}

void RAMPPIController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
    speed_limit_ = percentage ? speed_limit / 100.0 : speed_limit;
}

std::vector<ObstacleState> RAMPPIController::LoadObstacles(
    nav2_costmap_2d::Costmap2D* costmap) const
{
    std::vector<ObstacleState> obstacles;
    std::ifstream f("/tmp/cylinder_states");
    if (!f.is_open()) return obstacles;

    const float res = costmap->getResolution();
    const float r_hard = (OBS_RADIUS_M + OBS_HARD_MARGIN) / res;
    const float r_soft = OBS_SOFT_M / res;

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string name;
        float wx, wy, wvx, wvy;
        if (!(ss >> name >> wx >> wy >> wvx >> wvy)) continue;

        unsigned int omx, omy;
        if (!costmap->worldToMap(
                static_cast<double>(wx), static_cast<double>(wy), omx, omy))
            continue;

        ObstacleState obs;
        obs.row   = static_cast<float>(omy);
        obs.col   = static_cast<float>(omx);
        obs.v_row = wvy / res;
        obs.v_col = wvx / res;
        obs.r_hard = r_hard;
        obs.r_soft = r_soft;
        obstacles.push_back(obs);
    }
    return obstacles;
}

geometry_msgs::msg::TwistStamped RAMPPIController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist&,
    nav2_core::GoalChecker*)
{
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = node_.lock()->now();
    cmd.header.frame_id = pose.header.frame_id;

    if (plan_.poses.empty()) return cmd;

    if (!frames_logged_) {
        frames_logged_ = true;
        std::string plan_frame = plan_.header.frame_id;
        std::string plan_pose_frame = plan_.poses.empty() ? "<empty>"
                                                          : plan_.poses.front().header.frame_id;
        std::string plan_back_frame = plan_.poses.empty() ? "<empty>"
                                                          : plan_.poses.back().header.frame_id;
        RCLCPP_WARN(logger_,
            "[RAMPPI_FRAMES] pose.frame='%s' plan.header.frame='%s' "
            "plan.front.frame='%s' plan.back.frame='%s' "
            "costmap.global_frame='%s' "
            "pose=(%.3f,%.3f) plan_first=(%.3f,%.3f) plan_last=(%.3f,%.3f) "
            "plan_n=%zu",
            pose.header.frame_id.c_str(),
            plan_frame.c_str(),
            plan_pose_frame.c_str(),
            plan_back_frame.c_str(),
            costmap_ros_->getGlobalFrameID().c_str(),
            pose.pose.position.x, pose.pose.position.y,
            plan_.poses.front().pose.position.x, plan_.poses.front().pose.position.y,
            plan_.poses.back().pose.position.x, plan_.poses.back().pose.position.y,
            plan_.poses.size());
    }

    auto* costmap = costmap_ros_->getCostmap();
    double rx = pose.pose.position.x;
    double ry = pose.pose.position.y;

    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < plan_.poses.size(); ++i) {
        double dx = plan_.poses[i].pose.position.x - rx;
        double dy = plan_.poses[i].pose.position.y - ry;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < min_dist) { min_dist = d; closest_idx = i; }
    }
    double target_x = plan_.poses.back().pose.position.x;
    double target_y = plan_.poses.back().pose.position.y;
    for (size_t i = closest_idx; i < plan_.poses.size(); ++i) {
        double dx = plan_.poses[i].pose.position.x - rx;
        double dy = plan_.poses[i].pose.position.y - ry;
        if (std::sqrt(dx * dx + dy * dy) >= LOOKAHEAD) {
            target_x = plan_.poses[i].pose.position.x;
            target_y = plan_.poses[i].pose.position.y;
            break;
        }
    }

    unsigned int mx_r, my_r, mx_t, my_t;
    if (!costmap->worldToMap(rx, ry, mx_r, my_r)) {
        RCLCPP_WARN(logger_, "RAMPPIController: robot pose outside local costmap, stopping");
        return cmd;
    }
    if (!costmap->worldToMap(target_x, target_y, mx_t, my_t)) {
        double dx = target_x - rx;
        double dy = target_y - ry;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < 1e-6) return cmd;
        double max_r = costmap->getSizeInMetersX() * 0.45;
        target_x = rx + dx / d * max_r;
        target_y = ry + dy / d * max_r;
        if (!costmap->worldToMap(target_x, target_y, mx_t, my_t)) {
            return cmd;
        }
    }

    Field field(*costmap);
    RAMPPIConfig cfg;
    RAMPPI ramppi(cfg, ramppi_seed_++);

    std::pair<int,int> current{static_cast<int>(my_r), static_cast<int>(mx_r)};
    std::pair<int,int> target{static_cast<int>(my_t), static_cast<int>(mx_t)};

    std::vector<std::pair<int,int>> path_cells;
    path_cells.reserve(plan_.poses.size());
    for (const auto& p : plan_.poses) {
        unsigned int pmx, pmy;
        if (costmap->worldToMap(p.pose.position.x, p.pose.position.y, pmx, pmy)) {
            path_cells.emplace_back(static_cast<int>(pmy), static_cast<int>(pmx));
        }
    }

    auto obstacles = LoadObstacles(costmap);

    auto next_opt = ramppi.GetNextNode(current, target, field, path_cells,
                                      {prev_best_vx_, prev_best_vy_},
                                      obstacles, DT_STEP);
    if (!next_opt) {
        auto s = ramppi.GetLastStats();
        RCLCPP_WARN(logger_,
            "[RAMPPI_DIAG] NULLOPT rx=%.2f ry=%.2f costR=%u tx=%.2f ty=%.2f "
            "inv=%d/%d minC=%.1f meanC=%.1f maxW=%.3f",
            rx, ry, costmap->getCost(mx_r, my_r), target_x, target_y,
            s.num_invalid, s.num_samples, s.min_cost, s.mean_cost, s.weight_max);
        return cmd;
    }

    auto [vel_row, vel_col] = ramppi.GetLastVelocity();
    // Smooth MPPI output to suppress tick-to-tick noise
    constexpr float VEL_SMOOTH = 0.25f;
    float sv_row = VEL_SMOOTH * prev_best_vx_ + (1.0f - VEL_SMOOTH) * vel_row;
    float sv_col = VEL_SMOOTH * prev_best_vy_ + (1.0f - VEL_SMOOTH) * vel_col;
    prev_best_vx_ = sv_row;
    prev_best_vy_ = sv_col;

    // MPPI velocity in world frame: col→x, row→y
    double mppi_wx = static_cast<double>(sv_col);
    double mppi_wy = static_cast<double>(sv_row);
    double mppi_speed = std::sqrt(mppi_wx * mppi_wx + mppi_wy * mppi_wy);

    double yaw = tf2::getYaw(pose.pose.orientation);
    double vx_max = VX_MAX * speed_limit_;

    // Desired heading from MPPI; smooth it
    double desired_heading = (mppi_speed > 1e-4) ? std::atan2(mppi_wy, mppi_wx) : yaw;
    double smoothed_heading;
    if (!has_prev_angle_) {
        smoothed_heading = desired_heading;
        has_prev_angle_ = true;
    } else {
        double delta = std::atan2(
            std::sin(desired_heading - prev_target_angle_),
            std::cos(desired_heading - prev_target_angle_));
        smoothed_heading = prev_target_angle_ + ANGLE_SMOOTH_ALPHA * delta;
        smoothed_heading = std::atan2(std::sin(smoothed_heading), std::cos(smoothed_heading));
    }
    prev_target_angle_ = smoothed_heading;

    double heading_err = std::atan2(
        std::sin(smoothed_heading - yaw),
        std::cos(smoothed_heading - yaw));

    // Angular velocity
    cmd.twist.angular.z = std::clamp(KP_ANG * heading_err, -WZ_MAX, WZ_MAX);

    // Linear velocity: project MPPI onto robot forward axis.
    // Negative projection → robot backs up naturally.
    double fwd = mppi_wx * std::cos(yaw) + mppi_wy * std::sin(yaw);
    double turn_scale = std::max(0.2, 1.0 - std::abs(heading_err) / (M_PI / 2.0));
    double lin = std::clamp(fwd * KP_LIN, -1.0, 1.0) * vx_max * turn_scale;
    cmd.twist.linear.x = lin;

    static int diag_tick = 0;
    ++diag_tick;
    if (diag_tick % 3 == 0) {
        auto s = ramppi.GetLastStats();
        uint8_t cost_at_robot = costmap->getCost(mx_r, my_r);
        RCLCPP_INFO(logger_,
            "[RAMPPI_DIAG] t=%d rx=%.2f ry=%.2f yaw=%.2f tx=%.2f ty=%.2f "
            "costR=%u vrow=%.2f vcol=%.2f fwd=%.2f "
            "heading_err=%.2f lin=%.2f wz=%.2f "
            "inv=%d/%d minC=%.1f meanC=%.1f maxC=%.1f maxW=%.3f epsN=%.2f "
            "pathN=%zu",
            diag_tick, rx, ry, yaw, target_x, target_y,
            cost_at_robot, vel_row, vel_col, fwd,
            heading_err, lin, cmd.twist.angular.z,
            s.num_invalid, s.num_samples, s.min_cost, s.mean_cost, s.max_cost,
            s.weight_max, s.weighted_eps_norm,
            path_cells.size());
    }

    return cmd;
}

}
