#pragma once
#include <functional>
#include <optional>
#include <utility>
#include <vector>
#include "Field.h"

struct Result {
    std::vector<std::pair<int,int>> path;
    size_t iterations_count;
    size_t search_tree_size;
};

std::optional<Result> Astar(const std::pair<int, int> &start_pos,
                            const std::pair<int, int> &goal_pos,
                            Field &field,
                            const std::function<int(std::pair<int, int>,
                                                    std::pair<int, int>)>
                                    &heuristic_function);
