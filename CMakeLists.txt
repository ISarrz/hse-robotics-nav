cmake_minimum_required(VERSION 3.28)
project(dynamic_environment)

set(CMAKE_CXX_STANDARD 20)

add_executable(dynamic_environment main.cpp
        CliUtils.cpp
        CliUtils.h
        test_generation/Generators.cpp
        test_generation/Generators.h
        tests/StaticTests.cpp
        tests/StaticTests.h
        tests/DynamicTests.cpp
        tests/DynamicTests.h
        tests/TestUtils.h
        tests/MetricsUtils.h
        tests/LocalStaticTests.cpp
        tests/LocalStaticTests.h
        tests/LocalDynamicTests.cpp
        tests/LocalDynamicTests.h
        global_planning/Field.cpp
        global_planning/Field.h
        global_planning/heuristic_functions.cpp
        global_planning/heuristic_functions.h
        global_planning/Astar.cpp
        global_planning/Astar.h
        global_planning/DstarLite.cpp
        global_planning/DstarLite.h
        local_planning/MPPI.cpp
        local_planning/MPPI.h
        local_planning/RAMPPI.cpp
        local_planning/RAMPPI.h
        local_planning/MPPIPredict.cpp
        local_planning/MPPIPredict.h
        local_planning/RAMPPIPredict.cpp
        local_planning/RAMPPIPredict.h
        test_generation/TestGeneration.cpp
        test_generation/TestGeneration.h)

target_include_directories(dynamic_environment PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/global_planning
        ${CMAKE_CURRENT_SOURCE_DIR}/local_planning
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
        ${CMAKE_CURRENT_SOURCE_DIR}/test_generation)
