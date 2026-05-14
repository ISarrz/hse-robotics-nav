#pragma once
#include "MPPIPredict.h"
#include "RAMPPI.h"

struct RAMPPIPredictConfig : RAMPPIConfig {
    float predict_weight = 0.5f;
};

class RAMPPIPredict : public MPPI {
public:
    explicit RAMPPIPredict(RAMPPIPredictConfig config, unsigned int seed = 42);

    std::optional<std::pair<int, int>> GetNextNode(
        std::pair<int, int> current,
        std::pair<int, int> goal,
        const Field& field,
        const std::vector<DynamicObstacle>& obstacles);

private:
    RAMPPIPredictConfig rcfg_;
};
