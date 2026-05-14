#include "CliUtils.h"
#include "DynamicTests.h"
#include "Generators.h"
#include "LocalDynamicTests.h"
#include "LocalStaticTests.h"
#include "StaticTests.h"
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
int ParseNonNegativeInt(const std::string &value, const std::string &arg_name) {
    int parsed = 0;
    try {
        parsed = std::stoi(value);
    } catch (...) {
        throw std::runtime_error("Invalid " + arg_name + ": " + value);
    }
    if (parsed < 0) {
        throw std::runtime_error(arg_name + " must be non-negative");
    }
    return parsed;
}

AstarDynamicReplanMode ParseAstarDynamicMode(const std::string &mode) {
    if (mode == "step") {
        return AstarDynamicReplanMode::PerStep;
    }
    if (mode == "change") {
        return AstarDynamicReplanMode::PerChange;
    }
    throw std::runtime_error(
        "Invalid A* mode: " + mode +
        ". Expected one of: step, change");
}
}

int main(int argc, char **argv) {
    const int default_objects_per_step = 10;
    try {
        if (argc > 1) {
            std::string mode = argv[1];
            if (mode == "generate") {
                int steps = 2000;
                int objects_per_step = default_objects_per_step;

                if (argc >= 3) {
                    steps = ParsePositiveInt(argv[2], "steps");
                }
                if (argc >= 4) {
                    objects_per_step =
                        ParsePositiveInt(argv[3], "objects_per_step");
                }
                if (argc > 4) {
                    std::cerr << "Too many arguments for generate mode\n";
                    return 1;
                }

                GenerateDynamicChangesOnly(objects_per_step, steps);
                return 0;
            }
            if (mode == "generate-points") {
                int points_count = 100;
                int min_distance = -1;
                if (argc >= 3) {
                    points_count = ParsePositiveInt(argv[2], "points_count");
                }
                if (argc >= 4) {
                    min_distance =
                        ParseNonNegativeInt(argv[3], "min_distance");
                }
                if (argc > 4) {
                    std::cerr << "Too many arguments for generate-points mode\n";
                    return 1;
                }
                GeneratePointsOnly(points_count, min_distance);
                return 0;
            }
            if (mode == "test") {
                if (argc < 3 || argc > 4) {
                    std::cerr
                        << "Usage: ./dynamic_environment test [dynamic|static] [astar_mode]\n"
                        << "astar_mode for dynamic test: step|change (default: step)\n";
                    return 1;
                }
                std::string test_mode = argv[2];
                if (test_mode == "dynamic") {
                    AstarDynamicReplanMode astar_mode =
                        AstarDynamicReplanMode::PerStep;
                    if (argc == 4) {
                        astar_mode = ParseAstarDynamicMode(argv[3]);
                    }
                    DynamicTests(astar_mode);
                    return 0;
                }
                if (test_mode == "static") {
                    if (argc == 4) {
                        std::cerr << "A* mode is only supported for dynamic test\n";
                        return 1;
                    }
                    GetStaticTest();
                    return 0;
                }
                if (test_mode == "local-static") {
                    if (argc == 4) {
                        std::cerr << "No extra argument for local-static test\n";
                        return 1;
                    }
                    GetLocalStaticTest();
                    return 0;
                }
                if (test_mode == "local-dynamic") {
                    if (argc == 4) {
                        std::cerr << "No extra argument for local-dynamic test\n";
                        return 1;
                    }
                    LocalDynamicTests();
                    return 0;
                }
                std::cerr << "Unknown test mode: " << test_mode << "\n"
                          << "Usage: ./dynamic_environment test [dynamic|static|local-static|local-dynamic] [astar_mode]\n";
                return 1;
            }
            std::cerr << "Unknown mode: " << mode << "\n"
                      << "Usage: ./dynamic_environment [generate|generate-points|test]\n";
            return 1;
        }

        std::cout << "Usage:\n";
        std::cout << "  ./dynamic_environment generate [steps] [objects_per_step]\n";
        std::cout
            << "  ./dynamic_environment generate-points [count] [min_distance]\n";
        std::cout << "  ./dynamic_environment test [dynamic|static] [astar_mode]\n";
        std::cout << "    astar_mode for dynamic test: step|change (default: step)\n";
        std::cout << "Run 'generate' and 'generate-points' before tests.\n";
    } catch (const std::exception &e) {
        std::cerr << "Runtime error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
