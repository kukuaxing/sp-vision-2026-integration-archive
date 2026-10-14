#pragma once
#include <fstream>
#include <string>

inline bool raw_rmv_dataset(const std::string& folder) {
  std::ifstream source(folder + "/pose_source.txt");
  std::string value; source >> value;
  return value == "rmv_sensor1_frame1_wxyz";
}
