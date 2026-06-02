#pragma once
#include "field.hpp"
#include <optional>
#include <random>
#include <utility>
#include <vector>

struct ObstacleState {
    float row;
    float col;
    float v_row;
    float v_col;
    float r_hard;
    float r_soft;
};

struct MPPIConfig {
    int num_samples = 150;
    int horizon = 12;
    float lambda = 8.0f;
    float sigma = 1.0f;
    float obstacle_cost = 250.0f;
    float step_weight = 0.1f;
    float terminal_weight = 25.0f;
    float soft_obstacle_weight = 25.0f;
    float k_path = 1.5f;
    float warm_start_alpha = 0.5f;
};

struct MPPIStats {
    int num_samples = 0;
    int num_invalid = 0;
    float min_cost = 0.0f;
    float max_cost = 0.0f;
    float mean_cost = 0.0f;
    float weight_max = 0.0f;
    float weighted_eps_norm = 0.0f;
};

class MPPI {
public:
    explicit MPPI(MPPIConfig config, unsigned int seed = 42);
    virtual ~MPPI() = default;

    std::optional<std::pair<int, int>> GetNextNode(
        std::pair<int, int> current,
        std::pair<int, int> goal,
        const Field& field,
        const std::vector<std::pair<int, int>>& path_cells = {},
        std::pair<float, float> prior_mean = {0.0f, 0.0f},
        const std::vector<ObstacleState>& obstacles = {},
        float dt_step = 0.1f);

    std::pair<float, float> GetLastVelocity() const { return last_velocity_; }
    MPPIStats GetLastStats() const { return last_stats_; }

protected:
    virtual std::vector<float> ComputeWeights(const std::vector<float>& costs) const;

    MPPIConfig cfg_;
    std::mt19937 rng_;
    mutable std::vector<float> noise_buf_;
    std::pair<float, float> last_velocity_{0.0f, 0.0f};
    MPPIStats last_stats_{};
};
