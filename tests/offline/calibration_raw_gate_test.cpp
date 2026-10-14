#include "calibration/raw_pose_gate.hpp"
#include <cassert>
int main() {
  assert(!calibration_capture_allowed(true,false,false,false));
  assert(!calibration_capture_allowed(true,false,true,false));
  assert(calibration_capture_allowed(true,false,false,true));
  assert(calibration_capture_allowed(true,false,true,true));
  assert(!calibration_capture_allowed(false,true,false,true));
  assert(calibration_capture_allowed(false,true,true,false));
  assert(calibration_capture_allowed(false,false,false,false));
}
