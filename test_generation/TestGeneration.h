#pragma once

#include <string>
#include <utility>
#include <vector>

using Position = std::pair<int, int>;
using Change = std::pair<Position, Position>;
using StepChanges = std::vector<Change>;
using Changes = std::vector<StepChanges>;

struct ChangesHeader {
	int steps;
	int objects_per_step;
};

void SaveChangesToFile(const Changes &changes, const std::string &filename,
					   int objects_per_step);
ChangesHeader LoadChangesHeaderFromFile(const std::string &filename);
Changes LoadChangesFromFile(const std::string &filename);
void GetDynamicChanges(int steps = 2000, int objects_per_step = 1);
