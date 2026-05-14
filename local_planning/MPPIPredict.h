#pragma once
#include "MPPI.h"
#include <vector>

struct DynamicObstacle {
    int x, y;
    int vx, vy;
};

struct MPPIPredictConfig : MPPIConfig {
    float predict_weight = 0.5f;
};

class MPPIPredict : public MPPI {
public:
    explicit MPPIPredict(MPPIPredictConfig config, unsigned int seed = 42);

    std::optional<std::pair<int, int>> GetNextNode(
        std::pair<int, int> current,
        std::pair<int, int> goal,
        const Field& field,
        const std::vector<DynamicObstacle>& obstacles);

protected:
    MPPIPredictConfig pcfg_;
};
