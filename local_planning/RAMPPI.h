#pragma once
#include "MPPI.h"

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
        const Field& field);

private:
    RAMPPIConfig rcfg_;
};
