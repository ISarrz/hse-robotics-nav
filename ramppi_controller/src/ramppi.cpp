#include "ramppi_controller/ramppi.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

RAMPPI::RAMPPI(RAMPPIConfig config, unsigned int seed)
    : MPPI(config, seed), rcfg_(config) {}

std::optional<std::pair<int, int>> RAMPPI::GetNextNode(
    std::pair<int, int> current,
    std::pair<int, int> goal,
    const Field& field,
    const std::vector<std::pair<int, int>>& path_cells,
    std::pair<float, float> prior_mean,
    const std::vector<ObstacleState>& obstacles,
    float dt_step)
{
    auto [cx, cy] = current;
    auto [gx, gy] = goal;

    auto neighbors = field.GetNeighbours(cx, cy);
    if (neighbors.empty()) return std::nullopt;

    const int N = cfg_.num_samples;
    const int T = cfg_.horizon;
    const float H = static_cast<float>(field.GetHeight() - 1);
    const float W = static_cast<float>(field.GetWidth() - 1);

    float dx = static_cast<float>(gx - cx);
    float dy = static_cast<float>(gy - cy);
    float dist = std::sqrt(dx * dx + dy * dy);
    float nvx = (dist > 0.0f) ? dx / dist : 0.0f;
    float nvy = (dist > 0.0f) ? dy / dist : 0.0f;

    if (cfg_.warm_start_alpha > 0.0f &&
        (prior_mean.first != 0.0f || prior_mean.second != 0.0f)) {
        float a = cfg_.warm_start_alpha;
        nvx = a * prior_mean.first + (1.0f - a) * nvx;
        nvy = a * prior_mean.second + (1.0f - a) * nvy;
    }

    const int noise_count = N * T * 2;
    noise_buf_.resize(noise_count);
    std::uniform_real_distribution<float> unif(0.0f, 1.0f);
    for (int i = 0; i < noise_count; i += 2) {
        float u1 = unif(rng_) + 1e-10f;
        float u2 = unif(rng_);
        float r = cfg_.sigma * std::sqrt(-2.0f * std::log(u1));
        float th = 6.283185307f * u2;
        noise_buf_[i] = r * std::cos(th);
        noise_buf_[i + 1] = r * std::sin(th);
    }

    std::vector<float> costs(N);
    std::vector<float> eps_x(N), eps_y(N);
    int invalid_count = 0;

    for (int k = 0; k < N; ++k) {
        float x = static_cast<float>(cx);
        float y = static_cast<float>(cy);
        float cost = 0.0f;
        bool hit = false;

        const int base = k * T * 2;
        eps_x[k] = noise_buf_[base];
        eps_y[k] = noise_buf_[base + 1];

        for (int t = 0; t < T; ++t) {
            float ex = noise_buf_[base + t * 2];
            float ey = noise_buf_[base + t * 2 + 1];

            float vx = std::clamp(nvx + ex, -1.5f, 1.5f);
            float vy = std::clamp(nvy + ey, -1.5f, 1.5f);

            float nx = std::clamp(x + vx, 0.0f, H);
            float ny = std::clamp(y + vy, 0.0f, W);

            int ix = static_cast<int>(std::round(nx));
            int iy = static_cast<int>(std::round(ny));

            if (field.IsValid(ix, iy)) {
                x = nx;
                y = ny;
                cost += cfg_.soft_obstacle_weight *
                        static_cast<float>(field.GetCost(ix, iy)) / 254.0f;
            } else {
                cost += cfg_.obstacle_cost;
                hit = true;
            }

            if (!obstacles.empty()) {
                float t_ahead = static_cast<float>(t + 1) * dt_step;
                for (const auto& obs : obstacles) {
                    float pr = obs.row + obs.v_row * t_ahead;
                    float pc = obs.col + obs.v_col * t_ahead;
                    float dr = x - pr, dc = y - pc;
                    float dist = std::sqrt(dr * dr + dc * dc);
                    if (dist < obs.r_hard) {
                        cost += cfg_.obstacle_cost;
                        hit = true;
                    } else if (dist < obs.r_soft) {
                        float ratio = 1.0f - dist / obs.r_soft;
                        cost += cfg_.soft_obstacle_weight * ratio * ratio;
                    }
                }
            }

            float ddx = static_cast<float>(gx) - x;
            float ddy = static_cast<float>(gy) - y;
            cost += cfg_.step_weight * std::sqrt(ddx * ddx + ddy * ddy);

            if (!path_cells.empty() && cfg_.k_path > 0.0f) {
                float best_d2 = std::numeric_limits<float>::max();
                for (const auto& [pr, pc] : path_cells) {
                    float ep = x - static_cast<float>(pr);
                    float eq = y - static_cast<float>(pc);
                    float d2 = ep * ep + eq * eq;
                    if (d2 < best_d2) best_d2 = d2;
                }
                cost += cfg_.k_path * std::sqrt(best_d2);
            }
        }

        float tdx = static_cast<float>(gx) - x;
        float tdy = static_cast<float>(gy) - y;
        cost += cfg_.terminal_weight * std::sqrt(tdx * tdx + tdy * tdy);
        costs[k] = cost;
        if (hit) ++invalid_count;
    }

    const int ND = rcfg_.n_disturbed;
    std::normal_distribution<float> disturb(0.0f, rcfg_.disturb_sigma);
    std::vector<float> risk_costs(ND);

    for (int k = 0; k < N; ++k) {
        const int base = k * T * 2;

        for (int n = 0; n < ND; ++n) {
            float x = static_cast<float>(cx);
            float y = static_cast<float>(cy);
            float obs_cost = 0.0f;

            for (int t = 0; t < T; ++t) {
                float ex = noise_buf_[base + t * 2] + disturb(rng_);
                float ey = noise_buf_[base + t * 2 + 1] + disturb(rng_);

                float vx = std::clamp(nvx + ex, -1.5f, 1.5f);
                float vy = std::clamp(nvy + ey, -1.5f, 1.5f);

                float nx = std::clamp(x + vx, 0.0f, H);
                float ny = std::clamp(y + vy, 0.0f, W);

                int ix = static_cast<int>(std::round(nx));
                int iy = static_cast<int>(std::round(ny));

                if (field.IsValid(ix, iy)) {
                    x = nx;
                    y = ny;
                    obs_cost += cfg_.soft_obstacle_weight *
                                static_cast<float>(field.GetCost(ix, iy)) / 254.0f;
                } else {
                    obs_cost += cfg_.obstacle_cost;
                }

                if (!obstacles.empty()) {
                    float t_ahead = static_cast<float>(t + 1) * dt_step;
                    for (const auto& obs : obstacles) {
                        float pr = obs.row + obs.v_row * t_ahead;
                        float pc = obs.col + obs.v_col * t_ahead;
                        float dr = x - pr, dc = y - pc;
                        float dist = std::sqrt(dr * dr + dc * dc);
                        if (dist < obs.r_hard) {
                            obs_cost += cfg_.obstacle_cost;
                        } else if (dist < obs.r_soft) {
                            float ratio = 1.0f - dist / obs.r_soft;
                            obs_cost += cfg_.soft_obstacle_weight * ratio * ratio;
                        }
                    }
                }
            }
            risk_costs[n] = obs_cost;
        }

        std::sort(risk_costs.begin(), risk_costs.end(), std::greater<float>());
        int tail = std::max(1, static_cast<int>(std::round(ND * (1.0f - rcfg_.cvar_alpha))));
        float cvar = 0.0f;
        for (int n = 0; n < tail; ++n) cvar += risk_costs[n];
        cvar /= static_cast<float>(tail);

        if (cvar > rcfg_.cvar_threshold)
            costs[k] += rcfg_.cvar_weight * cvar;
    }

    auto weights = ComputeWeights(costs);

    float wex = 0.0f, wey = 0.0f;
    float weight_max = 0.0f;
    for (int k = 0; k < N; ++k) {
        wex += weights[k] * eps_x[k];
        wey += weights[k] * eps_y[k];
        if (weights[k] > weight_max) weight_max = weights[k];
    }

    float best_vx = std::clamp(nvx + wex, -1.5f, 1.5f);
    float best_vy = std::clamp(nvy + wey, -1.5f, 1.5f);
    last_velocity_ = {best_vx, best_vy};

    {
        float min_c = costs[0], max_c = costs[0], sum_c = 0.0f;
        for (float c : costs) {
            if (c < min_c) min_c = c;
            if (c > max_c) max_c = c;
            sum_c += c;
        }
        last_stats_.num_samples = N;
        last_stats_.num_invalid = invalid_count;
        last_stats_.min_cost = min_c;
        last_stats_.max_cost = max_c;
        last_stats_.mean_cost = sum_c / static_cast<float>(N);
        last_stats_.weight_max = weight_max;
        last_stats_.weighted_eps_norm = std::sqrt(wex * wex + wey * wey);
    }

    std::pair<int, int> best = neighbors[0];
    float best_d = std::numeric_limits<float>::max();
    for (auto [nx, ny] : neighbors) {
        float dnx = static_cast<float>(nx - cx) - best_vx;
        float dny = static_cast<float>(ny - cy) - best_vy;
        float d = dnx * dnx + dny * dny;
        if (d < best_d) {
            best_d = d;
            best = {nx, ny};
        }
    }
    return best;
}
