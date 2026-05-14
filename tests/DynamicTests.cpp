#include "DynamicTests.h"

#include "DataPaths.h"
#include "Astar.h"
#include "DstarLite.h"
#include "Field.h"
#include "TestGeneration.h"
#include "TestUtils.h"
#include "heuristic_functions.h"

#include <chrono>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr int kMaxSteps = 50'000;

struct PathRunResult {
    int status;
    int steps;
};

struct RunStats : BaseRunStats {
    int timeout = 0;
    int total_steps = 0;
    int min_steps = std::numeric_limits<int>::max();
    int max_steps = 0;

    void Add(const std::string &status, double ms, int steps) {
        if (status == "Timeout") {
            ++timeout;
        } else {
            RecordStatus(status);
        }
        RecordTiming(ms);
        total_steps += steps;
        if (steps < min_steps) min_steps = steps;
        if (steps > max_steps) max_steps = steps;
    }

    int TotalRuns() const { return BaseRunStats::TotalRuns() + timeout; }
};

void PrintStats(const std::string &label, const RunStats &stats) {
    const int total_runs = stats.TotalRuns();
    const double avg_ms = total_runs > 0 ? stats.total_ms / total_runs : 0.0;
    const double min_ms = total_runs > 0 ? stats.min_ms : 0.0;
    const double max_ms = total_runs > 0 ? stats.max_ms : 0.0;
    const double avg_steps =
        total_runs > 0 ? static_cast<double>(stats.total_steps) / total_runs : 0.0;
    const int min_steps = total_runs > 0 ? stats.min_steps : 0;
    const int max_steps = total_runs > 0 ? stats.max_steps : 0;

    std::cout << std::left << std::setw(14) << label << " | "
              << "ok=" << std::setw(4) << stats.success << " "
              << "no_path=" << std::setw(4) << stats.no_path << " "
              << "timeout=" << std::setw(4) << stats.timeout << " "
              << "exceptions=" << std::setw(4) << stats.exceptions << " "
              << std::fixed << std::setprecision(3)
              << "avg=" << std::setw(9) << avg_ms << "ms "
              << "min=" << std::setw(9) << min_ms << "ms "
              << "max=" << std::setw(9) << max_ms << "ms "
              << "avg_steps=" << std::setw(8) << avg_steps << " "
              << "min_steps=" << std::setw(5) << min_steps << " "
              << "max_steps=" << std::setw(5) << max_steps << "\n";
}

PathRunResult TestDynamicAstar(Field field, int x1, int y1, int x2, int y2,
                               const Changes &changes,
                               const AstarDynamicReplanMode astar_mode) {
    Coordinates current_pos = {x1, y1};
    Coordinates end_p = {x2, y2};
    int change_step = 0;
    int movement_steps = 0;

    while (current_pos != end_p) {
        if (movement_steps >= kMaxSteps) {
            return {-2, movement_steps};
        }

        std::optional<Result> result;

        if (change_step < static_cast<int>(changes.size())) {
            for (const auto &change : changes[change_step]) {
                field.Set(change.first.first, change.first.second, '.');
                field.Set(change.second.first, change.second.second, 'D');

                if (astar_mode == AstarDynamicReplanMode::PerChange) {
                    result = Astar(current_pos, end_p, field, ManhattanDistance);
                }
            }
            ++change_step;
        }

        if (!result.has_value()) {
            result = Astar(current_pos, end_p, field, ManhattanDistance);
        }

        if (!result.has_value() || result->path.size() < 2) {
            return {-1, movement_steps};
        }

        current_pos = result->path[1];
        ++movement_steps;
    }

    return {0, movement_steps};
}

PathRunResult TestDynamicDstarLite(Field field, int x1, int y1, int x2, int y2,
                                   const Changes &changes) {
    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};
    DstarLite dstar_lite = DstarLite(&field, start_p, end_p, ManhattanDistance);
    dstar_lite.ComputeShortestPath();

    Coordinates current_pos = start_p;
    int change_step = 0;
    int movement_steps = 0;
    while (current_pos != end_p) {
        if (movement_steps >= kMaxSteps) {
            return {-2, movement_steps};
        }

        bool has_map_updates = false;
        if (change_step < static_cast<int>(changes.size())) {
            std::vector<std::pair<Coordinates, char>> updates;
            updates.reserve(changes[change_step].size() * 2);
            for (const auto &change : changes[change_step]) {
                updates.push_back({change.first, '.'});
                updates.push_back({change.second, 'D'});
            }
            if (!updates.empty()) {
                dstar_lite.UpdateObstacles(updates);
                has_map_updates = true;
            }
            ++change_step;
        }

        if (has_map_updates) {
            dstar_lite.ComputeShortestPath();
        }

        auto next_node = dstar_lite.GetNextNode();
        if (!next_node.has_value() && !has_map_updates) {
            dstar_lite.ComputeShortestPath();
            next_node = dstar_lite.GetNextNode();
        }
        if (!next_node.has_value()) {
            return {-1, movement_steps};
        }
        dstar_lite.MoveStart({next_node->first, next_node->second});
        current_pos = {next_node->first, next_node->second};
        ++movement_steps;
    }

    return {0, movement_steps};
}

}

void DynamicTests(const AstarDynamicReplanMode astar_mode) {
    Field field(GetMapPath());
    ChangesHeader header = LoadChangesHeaderFromFile(GetChangesPath());
    Changes changes = LoadChangesFromFile(GetChangesPath());
    std::ifstream in(GetPointsPath());

    if (changes.empty()) {
        throw std::runtime_error("data/changes.txt does not contain any steps");
    }

    if (header.steps != static_cast<int>(changes.size())) {
        throw std::runtime_error("data/changes.txt header steps does not match data");
    }
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open data/points.txt");
    }

    std::cout << "Detected from header: steps=" << header.steps
              << ", objects_per_step=" << header.objects_per_step << "\n";

    int test_id = 1;
    RunStats a_star_stats;
    RunStats dstar_stats;

    std::cout << std::left << std::setw(6) << "Test" << "| "
              << std::setw(10) << "A*" << "| "
              << std::setw(10) << "A* time" << "| "
              << std::setw(9) << "A* steps" << "| "
              << std::setw(10) << "D* Lite" << "| "
              << std::setw(10) << "D* time" << "| "
              << std::setw(9) << "D* steps" << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    int x1, y1, x2, y2;
    while (in >> x1 >> y1 >> x2 >> y2) {
        std::string a_status = "Path found";
        double a_ms = 0.0;
        int a_steps = 0;

        auto a_start = std::chrono::high_resolution_clock::now();
        try {
            PathRunResult result =
                TestDynamicAstar(field, x1, y1, x2, y2, changes, astar_mode);
            a_steps = result.steps;

            if (result.status == -1) {
                a_status = "No path";
            } else if (result.status == -2) {
                a_status = "Timeout";
            }
        } catch (const std::exception &e) {
            a_status = "Exception";
        }
        auto a_end = std::chrono::high_resolution_clock::now();
        a_ms = std::chrono::duration<double, std::milli>(a_end - a_start).count();
        a_star_stats.Add(a_status, a_ms, a_steps);

        std::string d_status = "Path found";
        double d_ms = 0.0;
        int d_steps = 0;

        auto d_start = std::chrono::high_resolution_clock::now();
        try {
            PathRunResult result =
                TestDynamicDstarLite(field, x1, y1, x2, y2, changes);
            d_steps = result.steps;

            if (result.status == -1) {
                d_status = "No path";
            } else if (result.status == -2) {
                d_status = "Timeout";
            }
        } catch (const std::exception &e) {
            d_status = "Exception";
        }
        auto d_end = std::chrono::high_resolution_clock::now();
        d_ms = std::chrono::duration<double, std::milli>(d_end - d_start).count();
        dstar_stats.Add(d_status, d_ms, d_steps);

        std::cout << std::left << std::setw(6) << test_id << "| "
                  << std::setw(10) << a_status << "| "
                  << std::setw(10) << std::fixed << std::setprecision(3) << a_ms
                  << "| " << std::setw(9) << a_steps
                  << "| " << std::setw(10) << d_status << "| "
                  << std::setw(10) << std::fixed << std::setprecision(3) << d_ms
                  << "| " << std::setw(9) << d_steps
                  << "\n";

        ++test_id;
    }

    if (test_id == 1) {
        std::cout << "data/points.txt is empty or has invalid format.\n";
        return;
    }

    std::cout << "--------------------------------------------------------------------------------\n";
    PrintStats("A* Dynamic", a_star_stats);
    PrintStats("D* Dynamic", dstar_stats);
}
