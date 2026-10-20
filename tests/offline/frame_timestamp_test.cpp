#include "io/frame_timestamp.hpp"
#include <cassert>
#include <limits>

int main()
{
  using namespace std::chrono;
  const auto available = steady_clock::time_point(seconds(10));
  assert(io::estimated_exposure_midpoint(available, 6000) == available - milliseconds(3));
  assert(io::estimated_exposure_midpoint(available, 6000, 2000) == available - milliseconds(5));
  assert(io::estimated_exposure_midpoint(available, 0) == available);
  for (const double bad : {-1.0, 1000001.0, std::numeric_limits<double>::quiet_NaN()}) {
    bool rejected = false;
    try { (void)io::estimated_exposure_midpoint(available, bad); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  bool rejected = false;
  try { (void)io::estimated_exposure_midpoint(available, 6000, -1); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
}
