#include "textfileparser.h"
#include <iostream>
#include <fstream>
#include <sstream>

std::vector<std::vector<float>> readTxtFileTo2DVector(const std::string& filename) {
    std::vector<std::vector<float>> result;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return result;
    }

    std::string line;

    while (std::getline(file, line)) {
        std::vector<float> row;
        std::istringstream lineStream(line);
        float value;

        while (lineStream >> value) {
            row.push_back(value);
        }

        result.push_back(row);
    }

    file.close();
    return result;
}

// Function to perform bilinear interpolation
float dual_interpolate(std::vector<std::vector<float>>& table, float x, float y, float x_min, float x_max, float y_min, float y_max) {
    int cols = table[0].size();
    int rows = table.size();

    // Calculate the step sizes
    float x_step = float(x_max - x_min) / float((cols - 1.0));
    float y_step = float(y_max - y_min) / float((rows - 1.0));

    // Limit x within the boundaries
    if (x < x_min) {
        x = x_min;
    } else if (x > x_max) {
        x = x_max;
    }

    if (y < y_min) {
        y = y_min;
    } else if (x > y_max) {
        y = y_max;
    }
    // Normalize x and y to the index range
    float x_index = (x - x_min) / x_step;
    float y_index = (y - y_min) / y_step;

    // Find the neighboring indices
    int x0 = std::max(0, std::min(static_cast<int>(x_index), cols - 2));
    int x1 = x0 + 1;
    int y0 = std::max(0, std::min(static_cast<int>(y_index), rows - 2));
    int y1 = y0 + 1;

    // Perform bilinear interpolation
    float value = (table[y0][x0] * (x1 - x_index) * (y1 - y_index) +
                   table[y1][x0] * (x1 - x_index) * (y_index - y0) +
                   table[y0][x1] * (x_index - x0) * (y1 - y_index) +
                   table[y1][x1] * (x_index - x0) * (y_index - y0));

    return value;
}
