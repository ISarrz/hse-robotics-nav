#include "CliUtils.h"

#include <stdexcept>

int ParsePositiveInt(const std::string &value, const std::string &arg_name) {
    int parsed = 0;
    try {
        parsed = std::stoi(value);
    } catch (...) {
        throw std::runtime_error("Invalid " + arg_name + ": " + value);
    }
    if (parsed <= 0) {
        throw std::runtime_error(arg_name + " must be positive");
    }
    return parsed;
}
