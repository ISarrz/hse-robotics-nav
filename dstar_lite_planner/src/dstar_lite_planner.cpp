#include "dstar_lite_planner/dstar_lite_planner.hpp"
#include "dstar_lite_planner/field.hpp"
#include "dstar_lite_planner/dstar_lite.hpp"
#include "dstar_lite_planner/heuristic_functions.hpp"
#include <nav2_core/planner_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(dstar_lite_planner::DstarLitePlanner, nav2_core::GlobalPlanner)

namespace dstar_lite_planner {

void DstarLitePlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    name_ = name;
    costmap_ros_ = costmap_ros;
    RCLCPP_INFO(logger_, "DstarLitePlanner configured");
}

nav_msgs::msg::Path DstarLitePlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    std::function<bool()> cancel_checker)
{
    auto* costmap = costmap_ros_->getCostmap();

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_.lock()->now();

    unsigned int mx_s, my_s, mx_g, my_g;
    if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx_s, my_s)) {
        throw nav2_core::PlannerException("DstarLite: start outside costmap");
    }
    if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_g, my_g)) {
        throw nav2_core::PlannerException("DstarLite: goal outside costmap");
    }

    Coordinates grid_start{static_cast<int>(my_s), static_cast<int>(mx_s)};
    Coordinates grid_goal{static_cast<int>(my_g), static_cast<int>(mx_g)};

    if (grid_start == grid_goal) {
        geometry_msgs::msg::PoseStamped p;
        p.header = path.header;
        p.pose = goal.pose;
        path.poses.push_back(p);
        return path;
    }

    Field field(*costmap);

    if (!field.IsValid(grid_start.first, grid_start.second)) {
        throw nav2_core::PlannerException("DstarLite: start cell is obstacle");
    }
    if (!field.IsValid(grid_goal.first, grid_goal.second)) {
        throw nav2_core::PlannerException("DstarLite: goal cell is obstacle");
    }

    DstarLite dstar(&field, grid_start, grid_goal, ManhattanDistance);
    dstar.ComputeShortestPath();

    auto current = grid_start;
    const int max_steps = static_cast<int>(field.GetHeight() * field.GetWidth());

    for (int step = 0; step <= max_steps; ++step) {
        if (cancel_checker()) {
            throw nav2_core::PlannerException("DstarLite: planning cancelled");
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.orientation.w = 1.0;

        double wx, wy;
        costmap->mapToWorld(
            static_cast<unsigned int>(current.second),
            static_cast<unsigned int>(current.first),
            wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        path.poses.push_back(pose);

        if (current == grid_goal) break;

        auto next = dstar.GetNextNode();
        if (!next) {
            RCLCPP_WARN(logger_, "DstarLite: no path found from (%d,%d) to (%d,%d)",
                        grid_start.first, grid_start.second,
                        grid_goal.first, grid_goal.second);
            throw nav2_core::PlannerException("DstarLite: no path found");
        }
        dstar.MoveStart(*next);
        current = *next;
    }

    RCLCPP_INFO(logger_, "DstarLite: path found with %zu poses", path.poses.size());
    return path;
}

}
