#include "heuristic_functions.h"
#include <cmath>

int ManhattanDistance(const std::pair<int, int> &a,
                      const std::pair<int, int> &b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}