#include "TestGeneration.h"

#include "DataPaths.h"
#include "Field.h"

#include <array>
#include <fstream>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

namespace {
std::random_device rd;
std::mt19937 gen(rd());
}

void SaveChangesToFile(const Changes &changes, const std::string &filename,
                       const int objects_per_step) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + filename);
    }

    out << changes.size() << " " << objects_per_step << "\n";

    for (const auto &step_changes : changes) {
        out << step_changes.size() << "\n";
        for (const auto &ch : step_changes) {
            out << ch.first.first << " " << ch.first.second << " "
                << ch.second.first << " " << ch.second.second << "\n";
        }
    }
}

ChangesHeader LoadChangesHeaderFromFile(const std::string &filename) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + filename);
    }

    ChangesHeader header{};
    in >> header.steps >> header.objects_per_step;
    if (!in || header.steps < 0 || header.objects_per_step < 0) {
        throw std::runtime_error("Invalid file header in: " + filename);
    }

    return header;
}

Changes LoadChangesFromFile(const std::string &filename) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + filename);
    }

    int steps;
    int objects_per_step;
    in >> steps >> objects_per_step;

    if (!in || steps < 0 || objects_per_step < 0) {
        throw std::runtime_error("Invalid file header in: " + filename);
    }

    Changes changes;
    changes.reserve(steps);

    for (int i = 0; i < steps; i++) {
        int moves_count;
        in >> moves_count;
        if (!in || moves_count < 0) {
            throw std::runtime_error("Invalid step in: " + filename);
        }

        StepChanges step_changes;
        step_changes.reserve(moves_count);
        for (int j = 0; j < moves_count; j++) {
            Position a, b;
            in >> a.first >> a.second >> b.first >> b.second;

            if (!in) {
                throw std::runtime_error("File ended early: " + filename);
            }
            step_changes.push_back({a, b});
        }
        changes.push_back(step_changes);
    }

    return changes;
}

void GetDynamicChanges(const int steps, const int objects_per_step) {
    EnsureDataDir();
    Field field(GetMapPath());
    if (field.GetWidth() == 0 || field.GetHeight() == 0) {
        throw std::runtime_error("Map is empty");
    }
    if (objects_per_step < 1) {
        throw std::runtime_error("objects_per_step must be positive");
    }

    std::uniform_int_distribution<int> dist_rows(
        0, static_cast<int>(field.GetHeight()) - 1);
    std::uniform_int_distribution<int> dist_cols(
        0, static_cast<int>(field.GetWidth()) - 1);

    const std::array<Position, 4> deltas = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

    Changes changes;
    changes.reserve(steps);

    for (int i = 0; i < steps; i++) {
        StepChanges step_changes;
        step_changes.reserve(objects_per_step);

        std::set<Position> used_from;
        std::set<Position> used_to;

        for (int j = 0; j < objects_per_step; j++) {
            Change change{{-1, -1}, {-1, -1}};
            for (int attempt = 0; attempt < 1000 && change.first.first == -1;
                 attempt++) {
                int x = dist_rows(gen);
                int y = dist_cols(gen);

                if (field.Get(x, y) != 'D') {
                    continue;
                }
                if (used_from.count({x, y}) > 0) {
                    continue;
                }

                std::vector<Position> candidates;
                for (const auto &[dx, dy] : deltas) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (field.IsValid(nx, ny) && used_to.count({nx, ny}) == 0) {
                        candidates.push_back({nx, ny});
                    }
                }

                if (candidates.empty()) {
                    continue;
                }

                std::uniform_int_distribution<int> pick(
                    0, static_cast<int>(candidates.size()) - 1);
                Position next = candidates[pick(gen)];
                change = {{x, y}, next};
            }

            if (change.first.first == -1) {
                continue;
            }

            int x = change.first.first;
            int y = change.first.second;
            int dx = change.second.first;
            int dy = change.second.second;

            used_from.insert(change.first);
            used_to.insert(change.second);
            field.Set(x, y, '.');
            field.Set(dx, dy, 'D');
            step_changes.push_back(change);
        }

        changes.push_back(step_changes);
    }

    SaveChangesToFile(changes, GetChangesPath(), objects_per_step);
}
