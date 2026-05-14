#pragma once
#include "Field.h"
#include <cmath>
#include <utility>
#include <vector>

struct PathMetrics {
    double tortuosity = 1.0;
    double avg_turn_deg = 0.0;
    double near_miss_rate = 0.0;
    int steps = 0;
};

inline PathMetrics ComputePathMetrics(
    const std::vector<std::pair<int, int>>& path,
    int gx, int gy,
    const Field& field)
{
    PathMetrics m;
    if (path.size() < 2) return m;

    const int sx = path.front().first;
    const int sy = path.front().second;
    const double euclidean = std::sqrt(
        static_cast<double>((gx - sx) * (gx - sx) +
                            (gy - sy) * (gy - sy)));
    m.tortuosity = euclidean > 0.0
        ? static_cast<double>(path.size() - 1) / euclidean
        : 1.0;

    double total_angle = 0.0;
    int turns = 0;
    int near_miss = 0;
    constexpr int dx4[] = {1, -1, 0, 0};
    constexpr int dy4[] = {0, 0, 1, -1};

    for (int i = 1; i < static_cast<int>(path.size()); ++i) {
        auto [x, y] = path[i];
        for (int d = 0; d < 4; ++d) {
            if (!field.IsValid(x + dx4[d], y + dy4[d])) {
                ++near_miss;
                break;
            }
        }
        if (i >= 2) {
            auto [px, py] = path[i - 2];
            auto [cx, cy] = path[i - 1];
            auto [nx, ny] = path[i];
            const double v1x = cx - px, v1y = cy - py;
            const double v2x = nx - cx, v2y = ny - cy;
            const double len1 = std::sqrt(v1x * v1x + v1y * v1y);
            const double len2 = std::sqrt(v2x * v2x + v2y * v2y);
            if (len1 > 0.0 && len2 > 0.0) {
                double cosA = (v1x * v2x + v1y * v2y) / (len1 * len2);
                cosA = std::max(-1.0, std::min(1.0, cosA));
                total_angle += std::acos(cosA) * 180.0 / M_PI;
                ++turns;
            }
        }
    }

    m.steps = static_cast<int>(path.size()) - 1;
    m.near_miss_rate = static_cast<double>(near_miss) /
                       static_cast<double>(path.size() - 1);
    m.avg_turn_deg = turns > 0 ? total_angle / turns : 0.0;
    return m;
}
