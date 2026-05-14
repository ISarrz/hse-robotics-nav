#pragma once

enum class AstarDynamicReplanMode {
    PerStep,
    PerChange
};

void DynamicTests(AstarDynamicReplanMode astar_mode =
                      AstarDynamicReplanMode::PerStep);
