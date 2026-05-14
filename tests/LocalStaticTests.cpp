#include "LocalStaticTests.h"

#include "DataPaths.h"
#include "Field.h"
#include "MetricsUtils.h"
#include "MPPI.h"
#include "MPPIPredict.h"
#include "RAMPPI.h"
#include "RAMPPIPredict.h"
#include "TestUtils.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

constexpr int kMaxNavSteps = 500;
constexpr int kMaxTestCases = 100;

struct NavResult {
    std::string status;
    std::vector<std::pair<int, int>> path;
};

struct RunStats : BaseRunStats {
    long long total_steps = 0;
    int min_steps = std::numeric_limits<int>::max();
    int max_steps = 0;
    int step_samples = 0;
    int timeout = 0;

    double total_tort = 0.0, total_turn = 0.0, total_nm = 0.0;
    double total_steps_sq = 0.0;
    int metric_samples = 0;

    void Add(const std::string& status, double ms, const NavResult& res,
             const PathMetrics& pm) {
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
NavResult RunNavigationBase(
    Planner& planner, const Field& field_ref, int x1, int y1, int x2, int y2)
{
    Field field = field_ref;
    std::pair<int, int> current = {x1, y1};
    std::pair<int, int> goal = {x2, y2};

    NavResult result;
    result.path.push_back(current);

    if (current == goal) {
        result.status = "Path found";
        return result;
    }

    for (int steps = 0; steps < kMaxNavSteps; ++steps) {
        auto next = planner.GetNextNode(current, goal, field);
        if (!next.has_value()) {
            result.status = "No path";
            return result;
        }
        current = *next;
        result.path.push_back(current);
        if (current == goal) {
            result.status = "Path found";
            return result;
        }
    }
    result.status = "Timeout";
    return result;
}

template <typename Planner>
NavResult RunNavigationPredict(
    Planner& planner, const Field& field_ref, int x1, int y1, int x2, int y2)
{
    Field field = field_ref;
    std::pair<int, int> current = {x1, y1};
    std::pair<int, int> goal = {x2, y2};
    const std::vector<DynamicObstacle> empty_obstacles;

    NavResult result;
    result.path.push_back(current);

    if (current == goal) {
        result.status = "Path found";
        return result;
    }

    for (int steps = 0; steps < kMaxNavSteps; ++steps) {
        auto next = planner.GetNextNode(current, goal, field, empty_obstacles);
        if (!next.has_value()) {
            result.status = "No path";
            return result;
        }
        current = *next;
        result.path.push_back(current);
        if (current == goal) {
            result.status = "Path found";
            return result;
        }
    }
    result.status = "Timeout";
    return result;
}

template <typename Planner, typename RunFn>
void TimeOne(Planner& planner, RunFn run_fn, const Field& field,
             int x1, int y1, int x2, int y2,
             NavResult& res, PathMetrics& pm, double& ms,
             RunStats& stats)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    try {
        res = run_fn(planner, field, x1, y1, x2, y2);
        if (res.status == "Path found")
            pm = ComputePathMetrics(res.path, x2, y2, field);
    } catch (...) { res.status = "Exception"; }
    ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();
    stats.Add(res.status, ms, res, pm);
}

void PrintRow(const std::string& status, double ms, int steps, const PathMetrics& pm)
{
    std::cout << std::setw(10) << status << "| "
              << std::fixed << std::setprecision(3)
              << std::setw(10) << ms << "| "
              << std::setw(10) << steps << "| "
              << std::setprecision(3)
              << std::setw(10) << pm.tortuosity << "| "
              << std::setw(10) << pm.avg_turn_deg << "| "
              << std::setw(10) << pm.near_miss_rate << "| ";
}

}

void GetLocalStaticTest() {
    const Field field(GetMapPath());
    std::ifstream in(GetPointsPath());
    if (!in.is_open()) {
        std::cout << "Cannot open data/points.txt\n";
        return;
    }

    MPPIConfig mppi_cfg;
    RAMPPIConfig ramppi_cfg;
    MPPIPredictConfig mppi_p_cfg;
    RAMPPIPredictConfig ramppi_p_cfg;
    MPPI mppi(mppi_cfg, 42);
    RAMPPI ramppi(ramppi_cfg, 137);
    MPPIPredict mppi_p(mppi_p_cfg, 42);
    RAMPPIPredict ramppi_p(ramppi_p_cfg, 137);

    auto run_base_mppi    = [](MPPI&         p, const Field& f, int x1, int y1, int x2, int y2)
                              { return RunNavigationBase(p, f, x1, y1, x2, y2); };
    auto run_base_ramppi  = [](RAMPPI&       p, const Field& f, int x1, int y1, int x2, int y2)
                              { return RunNavigationBase(p, f, x1, y1, x2, y2); };
    auto run_pred_mppi    = [](MPPIPredict&  p, const Field& f, int x1, int y1, int x2, int y2)
                              { return RunNavigationPredict(p, f, x1, y1, x2, y2); };
    auto run_pred_ramppi  = [](RAMPPIPredict& p, const Field& f, int x1, int y1, int x2, int y2)
                              { return RunNavigationPredict(p, f, x1, y1, x2, y2); };

    RunStats mppi_stats, ramppi_stats, mppi_p_stats, ramppi_p_stats;
    int test_id = 1;

    std::cout << std::left << std::setw(6)  << "Test" << "| ";
    auto header_block = [](const std::string& name) {
        std::cout << std::setw(10) << name         << "| "
                  << std::setw(10) << (name+" ms") << "| "
                  << std::setw(10) << (name+" st") << "| "
                  << std::setw(10) << (name+" tt") << "| "
                  << std::setw(10) << (name+" tn") << "| "
                  << std::setw(10) << (name+" nm") << "| ";
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

        TimeOne(mppi,     run_base_mppi,   field, x1, y1, x2, y2, m_res,  m_pm,  m_ms,  mppi_stats);
        TimeOne(ramppi,   run_base_ramppi, field, x1, y1, x2, y2, r_res,  r_pm,  r_ms,  ramppi_stats);
        TimeOne(mppi_p,   run_pred_mppi,   field, x1, y1, x2, y2, mp_res, mp_pm, mp_ms, mppi_p_stats);
        TimeOne(ramppi_p, run_pred_ramppi, field, x1, y1, x2, y2, rp_res, rp_pm, rp_ms, ramppi_p_stats);

        const int m_steps  = m_res.status  == "Path found" ? static_cast<int>(m_res.path.size())  - 1 : 0;
        const int r_steps  = r_res.status  == "Path found" ? static_cast<int>(r_res.path.size())  - 1 : 0;
        const int mp_steps = mp_res.status == "Path found" ? static_cast<int>(mp_res.path.size()) - 1 : 0;
        const int rp_steps = rp_res.status == "Path found" ? static_cast<int>(rp_res.path.size()) - 1 : 0;

        std::cout << std::left << std::setw(6) << test_id << "| ";
        PrintRow(m_res.status,  m_ms,  m_steps,  m_pm);
        PrintRow(r_res.status,  r_ms,  r_steps,  r_pm);
        PrintRow(mp_res.status, mp_ms, mp_steps, mp_pm);
        PrintRow(rp_res.status, rp_ms, rp_steps, rp_pm);
        std::cout << "\n";

        ++test_id;
    }

    if (test_id == 1) {
        std::cout << "data/points.txt is empty or has invalid format.\n";
        return;
    }
    std::cout << std::string(300, '-') << "\n";
    PrintStats("MPPI Static",          mppi_stats);
    PrintStats("RAMPPI Static",        ramppi_stats);
    PrintStats("MPPIPredict Static",   mppi_p_stats);
    PrintStats("RAMPPIPredict Static", ramppi_p_stats);
}
