#include "dstar_lite_planner/field.hpp"
#include <fstream>
#include <stdexcept>
#include <sstream>

Field::Field(const std::string& file_name) {
    std::ifstream file(file_name);
    if (!file.is_open())
        throw std::runtime_error("Cannot open map file: " + file_name);

    std::string line;
    while (std::getline(file, line)) {
        std::vector<char> row;
        std::stringstream ss(line);
        char cell;
        while (ss >> cell) row.push_back(cell);
        if (!row.empty()) field_data_.push_back(row);
    }

    if (field_data_.empty())
        throw std::runtime_error("Map file is empty: " + file_name);

    height = field_data_.size();
    width = field_data_.front().size();
}

Field::Field(const nav2_costmap_2d::Costmap2D& costmap) {
    height = costmap.getSizeInCellsY();
    width = costmap.getSizeInCellsX();
    field_data_.assign(height, std::vector<char>(width, '.'));
    for (size_t r = 0; r < height; ++r) {
        for (size_t c = 0; c < width; ++c) {
            uint8_t cost = costmap.getCost(
                static_cast<unsigned int>(c),
                static_cast<unsigned int>(r));
            if (cost >= 253) field_data_[r][c] = '#';
        }
    }
}

char Field::Get(const size_t x, const size_t y) const {
    if (x >= height || y >= width)
        throw std::out_of_range("Field::Get out of bounds");
    return field_data_[x][y];
}

void Field::Set(const size_t x, const size_t y, const char value) {
    if (x >= height || y >= width)
        throw std::out_of_range("Field::Set out of bounds");
    field_data_[x][y] = value;
}

bool Field::IsValid(const int x, const int y) const {
    if (x < 0 || y < 0 || x >= static_cast<int>(height) || y >= static_cast<int>(width))
        return false;
    return Get(x, y) == '.';
}

std::vector<std::pair<int, int>> Field::GetNeighbours(const int x, const int y) const {
    std::vector<std::pair<int, int>> neighbours;
    for (auto [dx, dy] : std::vector<std::pair<int,int>>{{1,0},{0,1},{-1,0},{0,-1}}) {
        if (IsValid(x + dx, y + dy))
            neighbours.emplace_back(x + dx, y + dy);
    }
    return neighbours;
}

void Field::Draw() const {
    for (size_t r = 0; r < height; ++r) {
        for (size_t c = 0; c < width; ++c)
            std::cout << Get(r, c) << " ";
        std::cout << "\n";
    }
}

bool Field::CheckPath(const std::vector<std::pair<int, int>>& path) const {
    for (auto [x, y] : path)
        if (!IsValid(x, y)) return false;
    return true;
}

size_t Field::GetWidth() const { return width; }
size_t Field::GetHeight() const { return height; }
