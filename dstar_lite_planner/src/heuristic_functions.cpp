#include "dstar_lite_planner/heuristic_functions.hpp"
#include <cmath>

int ManhattanDistance(const std::pair<int, int>& a, const std::pair<int, int>& b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}
