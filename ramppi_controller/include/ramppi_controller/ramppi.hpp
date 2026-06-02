#pragma once
#include "mppi.hpp"

struct RAMPPIConfig : MPPIConfig {
    float cvar_alpha = 0.8f;
    int n_disturbed = 20;
    float disturb_sigma = 0.5f;
    float cvar_threshold = 0.0f;
    float cvar_weight = 1.0f;
};

class RAMPPI : public MPPI {
public:
    explicit RAMPPI(RAMPPIConfig config, unsigned int seed = 42);

    std::optional<std::pair<int, int>> GetNextNode(
        std::pair<int, int> current,
        std::pair<int, int> goal,
        const Field& field,
        const std::vector<std::pair<int, int>>& path_cells,
        std::pair<float, float> prior_mean,
        const std::vector<ObstacleState>& obstacles = {},
        float dt_step = 0.1f);

private:
    RAMPPIConfig rcfg_;
};
