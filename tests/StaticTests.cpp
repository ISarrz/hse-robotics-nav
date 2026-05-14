#include "StaticTests.h"

#include "DataPaths.h"
#include "Astar.h"
#include "DstarLite.h"
#include "Field.h"
#include "TestUtils.h"
#include "heuristic_functions.h"

#include <chrono>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

struct RunStats : BaseRunStats {
    long long total_path_len = 0;
    int min_path_len = std::numeric_limits<int>::max();
    int max_path_len = 0;
    int path_len_samples = 0;

    void Add(const std::string &status, double ms,
             std::optional<int> path_len) {
        RecordStatus(status);
        RecordTiming(ms);

        if (path_len.has_value()) {
            ++path_len_samples;
            total_path_len += *path_len;
            if (*path_len < min_path_len) min_path_len = *path_len;
            if (*path_len > max_path_len) max_path_len = *path_len;
        }
    }
};

void PrintStats(const std::string &label, const RunStats &stats) {
    const int total_runs = stats.TotalRuns();
    const double avg_ms = total_runs > 0 ? stats.total_ms / total_runs : 0.0;
    const double min_ms = total_runs > 0 ? stats.min_ms : 0.0;
    const double max_ms = total_runs > 0 ? stats.max_ms : 0.0;
    const double avg_len = stats.path_len_samples > 0
                               ? static_cast<double>(stats.total_path_len) /
                                     stats.path_len_samples
                               : 0.0;
    const int min_len = stats.path_len_samples > 0 ? stats.min_path_len : 0;
    const int max_len = stats.path_len_samples > 0 ? stats.max_path_len : 0;

    std::cout << std::left << std::setw(14) << label << " | "
              << "ok=" << std::setw(4) << stats.success << " "
              << "no_path=" << std::setw(4) << stats.no_path << " "
              << "exceptions=" << std::setw(4) << stats.exceptions << " "
              << std::fixed << std::setprecision(3)
              << "avg=" << std::setw(9) << avg_ms << "ms "
              << "min=" << std::setw(9) << min_ms << "ms "
              << "max=" << std::setw(9) << max_ms << "ms "
              << "len_avg=" << std::setw(8) << avg_len << " "
              << "len_min=" << std::setw(6) << min_len << " "
              << "len_max=" << std::setw(6) << max_len << "\n";
}

std::optional<int> TestAstar(Field field, int x1, int y1, int x2, int y2) {
    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};

    auto result = Astar(start_p, end_p, field, ManhattanDistance);
    if (!result.has_value()) {
        return std::nullopt;
    }

    return static_cast<int>(result->path.size()) - 1;
}

std::optional<int> TestDstarLite(Field field, int x1, int y1, int x2, int y2) {
    Coordinates start_p = {x1, y1};
    Coordinates end_p = {x2, y2};
    DstarLite dstar_lite = DstarLite(&field, start_p, end_p, ManhattanDistance);
    dstar_lite.ComputeShortestPath();

    Coordinates current_pos = start_p;
    int path_length = 0;

    while (current_pos != end_p) {
        auto next_node = dstar_lite.GetNextNode();
        if (!next_node.has_value()) {
            return std::nullopt;
        }
        dstar_lite.MoveStart({next_node->first, next_node->second});
        current_pos = {next_node->first, next_node->second};
        ++path_length;
    }

    return path_length;
}

}

void GetStaticTest() {
    Field field(GetMapPath());

    std::ifstream in(GetPointsPath());
    if (!in.is_open()) {
        std::cout << "Cannot open data/points.txt\n";
        return;
    }

    int test_id = 1;
    RunStats a_star_stats;
    RunStats dstar_stats;

    std::cout << std::left << std::setw(6) << "Test" << "| "
              << std::setw(10) << "A*" << "| "
              << std::setw(10) << "A* time" << "| "
              << std::setw(10) << "A* len" << "| "
              << std::setw(10) << "D* Lite" << "| "
              << std::setw(10) << "D* time" << "| "
              << std::setw(10) << "D* len" << "\n";
    std::cout << "---------------------------------------------------------------------------------\n";

    int x1, y1, x2, y2;
    while (in >> x1 >> y1 >> x2 >> y2) {
        std::string a_status = "Path found";
        double a_ms = 0.0;
        std::optional<int> a_path_len;

        auto a_start = std::chrono::high_resolution_clock::now();
        try {
            a_path_len = TestAstar(field, x1, y1, x2, y2);
            if (!a_path_len.has_value()) {
                a_status = "No path";
            }
        } catch (const std::exception &e) {
            a_status = "Exception";
        }
        auto a_end = std::chrono::high_resolution_clock::now();
        a_ms = std::chrono::duration<double, std::milli>(a_end - a_start).count();
        a_star_stats.Add(a_status, a_ms, a_path_len);

        std::string d_status = "Path found";
        double d_ms = 0.0;
        std::optional<int> d_path_len;

        auto d_start = std::chrono::high_resolution_clock::now();
        try {
            d_path_len = TestDstarLite(field, x1, y1, x2, y2);
            if (!d_path_len.has_value()) {
                d_status = "No path";
            }
        } catch (const std::exception &e) {
            d_status = "Exception";
        }
        auto d_end = std::chrono::high_resolution_clock::now();
        d_ms = std::chrono::duration<double, std::milli>(d_end - d_start).count();
        dstar_stats.Add(d_status, d_ms, d_path_len);

        std::cout << std::left << std::setw(6) << test_id << "| "
                  << std::setw(10) << a_status << "| "
                  << std::setw(10) << std::fixed << std::setprecision(3) << a_ms
                  << "| "
                  << std::setw(10)
                  << (a_path_len.has_value() ? std::to_string(*a_path_len)
                                             : std::string("-"))
                  << "| " << std::setw(10) << d_status << "| "
                  << std::setw(10) << std::fixed << std::setprecision(3) << d_ms
                  << "| "
                  << std::setw(10)
                  << (d_path_len.has_value() ? std::to_string(*d_path_len)
                                             : std::string("-"))
                  << "\n";

        ++test_id;
    }

    if (test_id == 1) {
        std::cout << "data/points.txt is empty or has invalid format.\n";
        return;
    }

    std::cout << "---------------------------------------------------------------------------------\n";
    PrintStats("A* Static", a_star_stats);
    PrintStats("D* Static", dstar_stats);
}
