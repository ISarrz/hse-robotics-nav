#pragma once

#include <filesystem>
#include <string>

inline std::string GetDataDirPath() {
    if (std::filesystem::exists("../CMakeLists.txt")) {
        return "../data";
    }
    return "data";
}

inline std::string GetMapPath() { return GetDataDirPath() + "/map.txt"; }
inline std::string GetPointsPath() { return GetDataDirPath() + "/points.txt"; }
inline std::string GetChangesPath() { return GetDataDirPath() + "/changes.txt"; }

inline void EnsureDataDir() {
    std::filesystem::create_directories(GetDataDirPath());
}
