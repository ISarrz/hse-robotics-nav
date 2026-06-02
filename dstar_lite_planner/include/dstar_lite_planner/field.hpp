#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>

class Field {
public:
    explicit Field(const std::string& file_name);
    explicit Field(const nav2_costmap_2d::Costmap2D& costmap);

    [[nodiscard]] char Get(size_t x, size_t y) const;
    void Set(size_t x, size_t y, char value);
    [[nodiscard]] bool IsValid(int x, int y) const;
    [[nodiscard]] std::vector<std::pair<int, int>> GetNeighbours(int x, int y) const;
    void Draw() const;
    [[nodiscard]] bool CheckPath(const std::vector<std::pair<int, int>>& path) const;
    size_t GetWidth() const;
    size_t GetHeight() const;

private:
    size_t width = 0;
    size_t height = 0;
    std::vector<std::vector<char>> field_data_;
};
