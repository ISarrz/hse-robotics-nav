#include "Generators.h"

#include "DataPaths.h"
#include "Field.h"
#include "TestGeneration.h"

#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <cstdlib>

namespace {
std::random_device rd;
std::mt19937 gen(rd());
}

void GeneratePointsOnly(const int points_count, const int min_distance) {
    if (points_count <= 0) {
        throw std::runtime_error("points_count must be positive");
    }
    if (min_distance < -1) {
        throw std::runtime_error("min_distance must be non-negative");
    }

    EnsureDataDir();
    Field field(GetMapPath());
    std::ofstream out(GetPointsPath());
    if (!out.is_open()) {
        throw std::runtime_error("Cannot open data/points.txt for writing");
    }

    std::uniform_int_distribution<int> dist_rows(
        0, static_cast<int>(field.GetHeight()) - 1);
    std::uniform_int_distribution<int> dist_cols(
        0, static_cast<int>(field.GetWidth()) - 1);

    for (int i = 0; i < points_count; ++i) {
        bool generated = false;
        for (int attempt = 0; attempt < 5000 && !generated; ++attempt) {
            int x1 = dist_rows(gen);
            int y1 = dist_cols(gen);
            int x2 = dist_rows(gen);
            int y2 = dist_cols(gen);

            if (x1 == x2 && y1 == y2) {
                continue;
            }
            if (field.Get(x1, y1) != '.' || field.Get(x2, y2) != '.') {
                continue;
            }
            if (min_distance >= 0) {
                const int manhattan_distance =
                    std::abs(x1 - x2) + std::abs(y1 - y2);
                if (manhattan_distance < min_distance) {
                    continue;
                }
            }

            out << x1 << " " << y1 << " " << x2 << " " << y2 << "\n";
            generated = true;
        }

        if (!generated) {
            throw std::runtime_error(
                "Failed to generate enough valid points for data/points.txt");
        }
    }

    std::cout << "Generated data/points.txt with " << points_count
              << " point pairs";
    if (min_distance >= 0) {
        std::cout << " (min Manhattan distance: " << min_distance << ")";
    }
    std::cout << "\n";
}

void GenerateDynamicChangesOnly(const int objects_per_step, const int steps) {
    GetDynamicChanges(steps, objects_per_step);
    std::cout << "Generated data/changes.txt with " << steps
              << " steps and up to "
              << objects_per_step << " moves per step\n";
}
