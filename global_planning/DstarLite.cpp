#include "DstarLite.h"

#include <array>
#include <unordered_set>

DstarNode::DstarNode(int x, int y) : x(x), y(y) {
    g = std::numeric_limits<float>::infinity();
    rhs = std::numeric_limits<float>::infinity();
}

DstarNode::DstarNode(int x, int y, float g, float rhs) : x(x), y(y), g(g), rhs(rhs) {}

DstarLite::DstarLite(
        Field *field_ptr, Coordinates start_pos, Coordinates goal_pos,
        const std::function<int(Coordinates, Coordinates)> &heuristic_function)
    : start_pos_(start_pos), goal_pos_(goal_pos), field_ptr_(field_ptr),
      heuristic_function_(heuristic_function) {

    km = 0;
    for (int i = 0; i < field_ptr->GetHeight(); ++i) {
        std::vector<DstarNode> line;
        for (int j = 0; j < field_ptr->GetWidth(); ++j) {
            line.emplace_back(i, j);
        }
        grid_.push_back(line);
    }

    DstarNode &goal_node = grid_[goal_pos_.first][goal_pos_.second];
    goal_node.rhs = 0;
    auto it = open_.insert({CalculateKey(&goal_node), &goal_node});
    goal_node.open_it = it.first;
    goal_node.in_open = true;
}

Key DstarLite::CalculateKey(DstarNode *node) {
    float min_g_rhs = std::min(node->g, node->rhs);
    float k1 = min_g_rhs + heuristic_function_(start_pos_, {node->x, node->y}) +
               km;
    float k2 = min_g_rhs;
    return {k1, k2};
}

void DstarLite::UpdateVertex(DstarNode *node) {
    if (Coordinates(node->x, node->y) != goal_pos_) {
        float min_rhs = std::numeric_limits<float>::infinity();
        for (auto next : field_ptr_->GetNeighbours(node->x, node->y)) {
            min_rhs = std::min(min_rhs, grid_[next.first][next.second].g + 1);
        }
        node->rhs = min_rhs;
    }

    if (node->in_open) {
        open_.erase(node->open_it);
        node->in_open = false;
    }

    if (node->g != node->rhs) {
        auto it = open_.insert({CalculateKey(node), node});
        node->open_it = it.first;
        node->in_open = true;
    }
}

void DstarLite::ComputeShortestPath() {
    DstarNode *start_node = &grid_[start_pos_.first][start_pos_.second];

    while (!open_.empty() && (open_.begin()->first < CalculateKey(start_node) ||
                              start_node->rhs != start_node->g)) {

        auto [k_old, u] = *open_.begin();
        open_.erase(open_.begin());
        u->in_open = false;
        Key k_new = CalculateKey(u);

        if (k_old < k_new) {
            auto it = open_.insert({k_new, u}).first;
            u->open_it = it;
            u->in_open = true;
        } else if (u->g > u->rhs) {
            u->g = u->rhs;
            for (auto n : field_ptr_->GetNeighbours(u->x, u->y)) {
                DstarNode *neighbor = &grid_[n.first][n.second];
                UpdateVertex(neighbor);
            }
        } else {
            u->g = std::numeric_limits<float>::infinity();
            UpdateVertex(u);
            for (auto n : field_ptr_->GetNeighbours(u->x, u->y)) {
                DstarNode *neighbor = &grid_[n.first][n.second];
                UpdateVertex(neighbor);
            }
        }
    }
}

std::optional<Coordinates> DstarLite::GetNextNode() {
    auto neighbours =
            field_ptr_->GetNeighbours(start_pos_.first, start_pos_.second);
    if (neighbours.empty())
        return std::nullopt;

    DstarNode *best_node = &grid_[neighbours[0].first][neighbours[0].second];
    float min_cost = std::numeric_limits<float>::infinity();

    for (auto n : neighbours) {
        DstarNode *neighbor = &grid_[n.first][n.second];
        float cost = 1 + neighbor->g;
        if (cost < min_cost) {
            min_cost = cost;
            best_node = neighbor;
        }
    }

    if (min_cost == std::numeric_limits<float>::infinity())
        return std::nullopt;
    return Coordinates{best_node->x, best_node->y};
}

void DstarLite::MoveStart(Coordinates new_pos) {
    km += heuristic_function_(start_pos_, new_pos);
    start_pos_ = new_pos;
}

void DstarLite::UpdateObstacle(Coordinates pos, char value) {
    UpdateObstacles({{{pos, value}}});
}

void DstarLite::UpdateObstacles(
    const std::vector<std::pair<Coordinates, char>> &updates) {
    if (updates.empty()) {
        return;
    }

    const int height = static_cast<int>(field_ptr_->GetHeight());
    const int width = static_cast<int>(field_ptr_->GetWidth());
    std::unordered_set<int> marked;
    marked.reserve(updates.size() * 5);
    std::vector<Coordinates> affected_nodes;
    affected_nodes.reserve(updates.size() * 5);

    const std::array<Coordinates, 5> deltas = {
        Coordinates{0, 0}, Coordinates{1, 0}, Coordinates{0, 1},
        Coordinates{-1, 0}, Coordinates{0, -1}};

    for (const auto &[pos, value] : updates) {
        if (field_ptr_->Get(pos.first, pos.second) == value) {
            continue;
        }

        field_ptr_->Set(pos.first, pos.second, value);

        for (const auto &[dx, dy] : deltas) {
            const int nx = pos.first + dx;
            const int ny = pos.second + dy;
            if (nx < 0 || nx >= height || ny < 0 || ny >= width) {
                continue;
            }

            const int flat = nx * width + ny;
            if (marked.insert(flat).second) {
                affected_nodes.push_back({nx, ny});
            }
        }
    }

    for (const auto &node_pos : affected_nodes) {
        UpdateVertex(&grid_[node_pos.first][node_pos.second]);
    }
}