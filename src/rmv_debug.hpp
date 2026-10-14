#pragma once
#include <string>
int run_rmv_debug(const std::string& config_path, bool follow, double seconds, bool check_only,
  bool rotating=false,bool fire=false);
int run_rmv_shot_test(const std::string& config_path,unsigned count,double seconds,bool check_only,bool rpm_measurement=false);
