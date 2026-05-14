#pragma once
#include "Field.h"
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <set>
#include <tuple>
#include <vector>
using Key = std::pair<float, float>;
using Coordinates = std::pair<int, int>;

class DstarNode {
  public:
    int x, y;
    float g, rhs;
    bool in_open = false;
    std::set<std::pair<Key, DstarNode *>>::iterator open_it;

    DstarNode(int x, int y);
    DstarNode(int x, int y, float g, float rhs);
};

struct Compare {
    bool operator()(const std::pair<Key, DstarNode *> &a,
                    const std::pair<Key, DstarNode *> &b) const {
        if (a.first.first != b.first.first)
            return a.first.first < b.first.first;
        if (a.first.second != b.first.second)
            return a.first.second < b.first.second;
        return a.second < b.second;
    }
};

class DstarLite {
  public:
    DstarLite(Field *field_ptr, Coordinates start_pos, Coordinates goal_pos,
              const std::function<int(Coordinates, Coordinates)>
                      &heuristic_function);

    Key CalculateKey(DstarNode *);
    void UpdateVertex(DstarNode *);
    void ComputeShortestPath();
    std::optional<Coordinates> GetNextNode();
    void MoveStart(Coordinates new_pos);
    void UpdateObstacle(Coordinates pos, char value);
    void UpdateObstacles(const std::vector<std::pair<Coordinates, char>> &updates);

  private:
    float km = 0;
    std::function<int(Coordinates, Coordinates)> heuristic_function_;
    Coordinates start_pos_;
    Coordinates goal_pos_;
    Field *field_ptr_;
    std::vector<std::vector<DstarNode>> grid_;
    std::set<std::pair<Key, DstarNode *>, Compare> open_;
};