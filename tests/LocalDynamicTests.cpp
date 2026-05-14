#include "LocalDynamicTests.h"

#include "DataPaths.h"
#include "Field.h"
#include "MetricsUtils.h"
#include "MPPI.h"
#include "MPPIPredict.h"
#include "RAMPPI.h"
#include "RAMPPIPredict.h"
#include "TestGeneration.h"
#include "TestUtils.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr int kMaxSteps = 50'000;
constexpr int kMaxTestCases = 100;

struct NavResult {
    int status;
    std::vector<std::pair<int, int>> path;
};

struct RunStats : BaseRunStats {
    int timeout = 0;
    long long total_steps = 0;
    int min_steps = std::numeric_limits<int>::max();
    int max_steps = 0;
    int step_samples = 0;

    double total_tort = 0.0, total_turn = 0.0, total_nm = 0.0;
    double total_steps_sq = 0.0;
    int metric_samples = 0;

    void Add(const std::string& status, double ms,
             const NavResult& res, const PathMetrics& pm) {
        if (status == "Timeout") {
            ++timeout;
        } else {
            RecordStatus(status);
        }
        RecordTiming(ms);
        if (status == "Path found" && !res.path.empty()) {
            int steps = static_cast<int>(res.path.size()) - 1;
            ++step_samples;
            total_steps += steps;
            total_steps_sq += static_cast<double>(steps) * steps;
            if (steps < min_steps) min_steps = steps;
            if (steps > max_steps) max_steps = steps;

            ++metric_samples;
            total_tort += pm.tortuosity;
            total_turn += pm.avg_turn_deg;
            total_nm += pm.near_miss_rate;
        }
    }

    int TotalRuns() const { return BaseRunStats::TotalRuns() + timeout; }
};

void PrintStats(const std::string& label, const RunStats& s) {
    const int total = s.TotalRuns();
    const double avg_ms = total > 0 ? s.total_ms / total : 0.0;
    const double avg_steps = s.step_samples > 0
        ? static_cast<double>(s.total_steps) / s.step_samples : 0.0;
    const double avg_steps_sq = s.step_samples > 0
        ? s.total_steps_sq / s.step_samples : 0.0;
    const double steps_std = s.step_samples > 1
        ? std::sqrt(std::max(0.0, avg_steps_sq - avg_steps * avg_steps)) : 0.0;
    const double avg_tort = s.metric_samples > 0 ? s.total_tort / s.metric_samples : 0.0;
    const double avg_turn = s.metric_samples > 0 ? s.total_turn / s.metric_samples : 0.0;
    const double avg_nm = s.metric_samples > 0 ? s.total_nm / s.metric_samples : 0.0;

    std::cout << std::left << std::setw(18) << label << " | "
              << "ok=" << std::setw(4) << s.success << " "
              << "no_path=" << std::setw(4) << s.no_path << " "
              << "timeout=" << std::setw(4) << s.timeout << " "
              << std::fixed << std::setprecision(3)
              << "avg_ms=" << std::setw(8) << avg_ms << " "
              << "avg_steps=" << std::setw(7) << avg_steps << " "
              << "steps_std=" << std::setw(7) << steps_std << " "
              << "tortuosity=" << std::setw(6) << avg_tort << " "
              << "turn_deg=" << std::setw(6) << avg_turn << " "
              << "near_miss=" << std::setw(6) << avg_nm << "\n";
}

template <typename Planner>
NavResult TestPlannerNoPredict(
    Planner& planner, Field field,
    int x1, int y1, int x2, int y2,
    const Changes& changes)
{
    std::pair<int, int> current = {x1, y1};
    std::pair<int, int> goal = {x2, y2};
    int change_step = 0;

    NavResult result;
    result.path.push_back(current);

    while (current != goal) {
        if (static_cast<int>(result.path.size()) - 1 >= kMaxSteps) {
            result.status = -2;
            return result;
        }

        if (change_step < static_cast<int>(changes.size())) {
            for (const auto& change : changes[change_step]) {
                field.Set(change.first.first, change.first.second, '.');
                field.Set(change.second.first, change.second.second, 'D');
            }
            ++change_step;
        }

        auto next = planner.GetNextNode(current, goal, field);
        if (!next.has_value()) {
            result.status = -1;
            return result;
        }
        current = *next;
        result.path.push_back(current);
    }
    result.status = 0;
    return result;
}

template <typename Planner>
NavResult TestPlannerWithPredict(
    Planner& planner, Field field,
    int x1, int y1, int x2, int y2,
    const Changes& changes)
{
    std::pair<int, int> current = {x1, y1};
    std::pair<int, int> goal = {x2, y2};
    int change_step = 0;
    std::vector<DynamicObstacle> obstacles;

    NavResult result;
    result.path.push_back(current);

    while (current != goal) {
        if (static_cast<int>(result.path.size()) - 1 >= kMaxSteps) {
            result.status = -2;
            return result;
        }

        if (change_step < static_cast<int>(changes.size())) {
            obstacles.clear();
            obstacles.reserve(changes[change_step].size());
            for (const auto& change : changes[change_step]) {
                field.Set(change.first.first, change.first.second, '.');
                field.Set(change.second.first, change.second.second, 'D');
                DynamicObstacle obs;
                obs.x = change.second.first;
                obs.y = change.second.second;
                obs.vx = change.second.first - change.first.first;
                obs.vy = change.second.second - change.first.second;
                obstacles.push_back(obs);
            }
            ++change_step;
        }

        auto next = planner.GetNextNode(current, goal, field, obstacles);
        if (!next.has_value()) {
            result.status = -1;
            return result;
        }
        current = *next;
        result.path.push_back(current);
    }
    result.status = 0;
    return result;
}

std::string StatusToString(int status) {
    switch (status) {
        case 0:  return "Path found";
        case -1: return "No path";
        case -2: return "Timeout";
        default: return "Exception";
    }
}

template <typename Planner, typename TestFn>
void RunPlanner(Planner& planner, TestFn test_fn, const Field& field_orig,
                int x1, int y1, int x2, int y2, const Changes& changes,
                NavResult& res, PathMetrics& pm, double& ms,
                RunStats& stats)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    try {
        res = test_fn(planner, field_orig, x1, y1, x2, y2, changes);
        if (res.status == 0)
            pm = ComputePathMetrics(res.path, x2, y2, field_orig);
    } catch (...) { res.status = -3; }
    ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();
    stats.Add(StatusToString(res.status), ms, res, pm);
}

void PrintRow(int test_id,
              const std::string& status, double ms, int steps, const PathMetrics& pm)
{
    std::cout << std::setw(10) << status << "| "
              << std::fixed << std::setprecision(3)
              << std::setw(10) << ms << "| "
              << std::setw(10) << steps << "| "
              << std::setprecision(3)
              << std::setw(10) << pm.tortuosity << "| "
              << std::setw(10) << pm.avg_turn_deg << "| "
              << std::setw(10) << pm.near_miss_rate << "| ";
    (void)test_id;
}

}

void LocalDynamicTests() {
    const Field field_orig(GetMapPath());
    ChangesHeader header = LoadChangesHeaderFromFile(GetChangesPath());
    Changes changes = LoadChangesFromFile(GetChangesPath());
    std::ifstream in(GetPointsPath());

    if (changes.empty())
        throw std::runtime_error("data/changes.txt does not contain any steps");
    if (header.steps != static_cast<int>(changes.size()))
        throw std::runtime_error("data/changes.txt header steps does not match data");
    if (!in.is_open())
        throw std::runtime_error("Cannot open data/points.txt");

    std::cout << "Detected from header: steps=" << header.steps
              << ", objects_per_step=" << header.objects_per_step << "\n";

    MPPIConfig mppi_cfg;
    RAMPPIConfig ramppi_cfg;
    MPPIPredictConfig mppi_p_cfg;
    RAMPPIPredictConfig ramppi_p_cfg;

    MPPI mppi(mppi_cfg, 42);
    RAMPPI ramppi(ramppi_cfg, 137);
    MPPIPredict mppi_p(mppi_p_cfg, 42);
    RAMPPIPredict ramppi_p(ramppi_p_cfg, 137);

    RunStats mppi_stats, ramppi_stats, mppi_p_stats, ramppi_p_stats;
    int test_id = 1;

    std::cout << std::left
              << std::setw(6)  << "Test"          << "| ";
    auto header_block = [](const std::string& name) {
        std::cout << std::setw(10) << name        << "| "
                  << std::setw(10) << (name+" ms")<< "| "
                  << std::setw(10) << (name+" st")<< "| "
                  << std::setw(10) << (name+" tt")<< "| "
                  << std::setw(10) << (name+" tn")<< "| "
                  << std::setw(10) << (name+" nm")<< "| ";
    };
    header_block("MPPI");
    header_block("RAMPPI");
    header_block("MPPIp");
    header_block("RAMPPIp");
    std::cout << "\n" << std::string(300, '-') << "\n";
    std::cout << "(Using first " << kMaxTestCases
              << " test cases — sampling-based planners are slower per path)\n";

    int x1, y1, x2, y2;
    while (test_id <= kMaxTestCases && (in >> x1 >> y1 >> x2 >> y2)) {
        NavResult m_res, r_res, mp_res, rp_res;
        PathMetrics m_pm, r_pm, mp_pm, rp_pm;
        double m_ms = 0.0, r_ms = 0.0, mp_ms = 0.0, rp_ms = 0.0;

        RunPlanner(mppi,    TestPlannerNoPredict<MPPI>,
                   field_orig, x1, y1, x2, y2, changes,
                   m_res, m_pm, m_ms, mppi_stats);
        RunPlanner(ramppi,  TestPlannerNoPredict<RAMPPI>,
                   field_orig, x1, y1, x2, y2, changes,
                   r_res, r_pm, r_ms, ramppi_stats);
        RunPlanner(mppi_p,  TestPlannerWithPredict<MPPIPredict>,
                   field_orig, x1, y1, x2, y2, changes,
                   mp_res, mp_pm, mp_ms, mppi_p_stats);
        RunPlanner(ramppi_p, TestPlannerWithPredict<RAMPPIPredict>,
                   field_orig, x1, y1, x2, y2, changes,
                   rp_res, rp_pm, rp_ms, ramppi_p_stats);

        const int m_steps  = m_res.status  == 0 ? static_cast<int>(m_res.path.size())  - 1 : 0;
        const int r_steps  = r_res.status  == 0 ? static_cast<int>(r_res.path.size())  - 1 : 0;
        const int mp_steps = mp_res.status == 0 ? static_cast<int>(mp_res.path.size()) - 1 : 0;
        const int rp_steps = rp_res.status == 0 ? static_cast<int>(rp_res.path.size()) - 1 : 0;

        std::cout << std::left << std::setw(6) << test_id << "| ";
        PrintRow(test_id, StatusToString(m_res.status),  m_ms,  m_steps,  m_pm);
        PrintRow(test_id, StatusToString(r_res.status),  r_ms,  r_steps,  r_pm);
        PrintRow(test_id, StatusToString(mp_res.status), mp_ms, mp_steps, mp_pm);
        PrintRow(test_id, StatusToString(rp_res.status), rp_ms, rp_steps, rp_pm);
        std::cout << "\n";

        ++test_id;
    }

    if (test_id == 1) {
        std::cout << "data/points.txt is empty or has invalid format.\n";
        return;
    }
    std::cout << std::string(300, '-') << "\n";
    PrintStats("MPPI Dynamic",         mppi_stats);
    PrintStats("RAMPPI Dynamic",       ramppi_stats);
    PrintStats("MPPIPredict Dynamic",  mppi_p_stats);
    PrintStats("RAMPPIPredict Dynamic", ramppi_p_stats);
}
