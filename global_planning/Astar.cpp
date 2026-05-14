#include "Astar.h"

#include <algorithm>
#include <limits>

#include <queue>
#include <vector>

struct Node {
    int x, y;
    int f, g;

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

std::optional<Result> Astar(
    const std::pair<int, int>& start_pos,
    const std::pair<int, int>& goal_pos,
    Field& field,
    const std::function<int(std::pair<int, int>, std::pair<int, int>)>& heuristic_function) {

    int width = field.GetWidth();
    int height = field.GetHeight();

    std::vector<int> g_score(width * height, std::numeric_limits<int>::max());
    std::vector<int> parent_idx(width * height, -1);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;

    int start_idx = start_pos.first * width + start_pos.second;
    g_score[start_idx] = 0;
    open_set.push({start_pos.first, start_pos.second, heuristic_function(start_pos, goal_pos), 0});

    size_t steps = 0;
    size_t visited_count = 0;

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        int curr_idx = current.x * width + current.y;

        if (current.g > g_score[curr_idx]) continue;

        visited_count++;

        if (current.x == goal_pos.first && current.y == goal_pos.second) {
            std::vector<std::pair<int, int>> path;
            int p = curr_idx;
            while (p != -1) {
                path.push_back({p / width, p % width});
                p = parent_idx[p];
            }
            std::reverse(path.begin(), path.end());
            return Result(path, steps, visited_count);
        }

        for (auto [nx, ny] : field.GetNeighbours(current.x, current.y)) {
            int next_idx = nx * width + ny;
            int tentative_g = current.g + 1;

            if (tentative_g < g_score[next_idx]) {
                parent_idx[next_idx] = curr_idx;
                g_score[next_idx] = tentative_g;
                int h = heuristic_function({nx, ny}, goal_pos);
                open_set.push({nx, ny, tentative_g + h, tentative_g});
            }
        }
        steps++;
    }

    return std::nullopt;
}

