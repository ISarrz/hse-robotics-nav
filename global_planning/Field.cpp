#include "Field.h"
#include <fstream>
#include <stdexcept>
#include <sstream>

Field::Field(const std::string &file_name) {
    field_data_.clear();
    std::ifstream file(file_name);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open map file: " + file_name);
    }

    std::string line;
    while (getline(file, line)) {
        std::vector<char> row;
        std::stringstream ss(line);
        char cell;

        while (ss >> cell) {
            row.push_back(cell);
        }

        if (!row.empty()) {
            field_data_.push_back(row);
        }
    }

    if (field_data_.empty()) {
        throw std::runtime_error("Map file is empty or invalid: " + file_name);
    }

    const size_t expected_width = field_data_.front().size();
    for (const auto &row : field_data_) {
        if (row.size() != expected_width) {
            throw std::runtime_error("Map rows have inconsistent widths in: " +
                                     file_name);
        }
    }

    height = field_data_.size();
    width = expected_width;

    file.close();
}

char Field::Get(const size_t x, const size_t y) const {
    if (x >= height || y >= width) {
        throw std::out_of_range("Field::Get coordinates are out of bounds");
    }
    return field_data_[x][y];
}
void Field::Set(const size_t x, const size_t y, const char value) {
    if (x >= height || y >= width) {
        throw std::out_of_range("Field::Set coordinates are out of bounds");
    }
    field_data_[x][y] = value;
}

bool Field::IsValid(const int x, const int y) const {
    if (x < 0 || y < 0 || x >= static_cast<int>(height) ||
        y >= static_cast<int>(width)) {
        return false;
    }
    if (Get(x, y) != '.') {
        return false;
    }

    return true;
}

std::vector<std::pair<int, int>> Field::GetNeighbours(const int x,
                                                      const int y) const {
    std::vector<std::pair<int, int>> neighbours;
    std::vector<std::pair<int, int>> deltas = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    for (auto [dx, dy] : deltas) {
        const int i = dx + x;
        const int j = dy + y;
        if (IsValid(i, j)) {
            neighbours.emplace_back(i, j);
        }
    }

    return neighbours;
}

void Field::Draw() const {
    for (size_t row = 0; row < height; row++) {
        for (size_t column = 0; column < width; column++) {
            std::cout << Get(row, column) << " ";
        }
        std::cout << std::endl;
    }
}

bool Field::CheckPath(const std::vector<std::pair<int, int>> &path) const {
    for (auto [x, y] : path) {
        if (!IsValid(x, y)) {
            return false;
        }
    }
    return true;
}
size_t Field::GetWidth() const { return width; }
size_t Field::GetHeight() const { return height; }