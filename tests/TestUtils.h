#pragma once
#include <limits>
#include <string>

struct BaseRunStats {
    int success = 0;
    int no_path = 0;
    int exceptions = 0;
    double total_ms = 0.0;
    double min_ms = std::numeric_limits<double>::infinity();
    double max_ms = 0.0;

    void RecordStatus(const std::string &status) {
        if (status == "Path found") ++success;
        else if (status == "No path") ++no_path;
        else ++exceptions;
    }

    void RecordTiming(double ms) {
        total_ms += ms;
        if (ms < min_ms) min_ms = ms;
        if (ms > max_ms) max_ms = ms;
    }

    int TotalRuns() const { return success + no_path + exceptions; }
};
