#ifndef TEXTFILEPARSER_H
#define TEXTFILEPARSER_H

#include <vector>
#include <string>

std::vector<std::vector<float>> readTxtFileTo2DVector(const std::string& filename);
float dual_interpolate(std::vector<std::vector<float>>& table, float x, float y, float x_min, float x_max, float y_min, float y_max);


#endif // TEXTFILEPARSER_H
