#pragma once
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace fixed_sim {
// Fixture clock only. Recorded OBSERVATION intervals are a sensitivity input,
// not a measurement of capture age. Last element is a boundary/deadline.
inline std::vector<double> capture_offsets(double duration,double fps,
    const std::vector<double>& intervals={},std::size_t offset=0) {
  if(!std::isfinite(duration)||duration<=0||duration>300||!std::isfinite(fps)||fps<5||fps>500)
    throw std::invalid_argument("invalid capture clock bounds");
  std::vector<double> times;
  if(intervals.empty()) {
    const int n=static_cast<int>(std::ceil(duration*fps));
    for(int i=0;i<=n;++i)times.push_back(i/fps); // preserve historical exact arithmetic
    return times;
  }
  if(intervals.size()>1000000)throw std::invalid_argument("excessive cadence input");
  for(double dt:intervals)if(!std::isfinite(dt)||dt<.001||dt>.5)
    throw std::invalid_argument("cadence intervals must be finite, 1..500ms");
  times.push_back(0.);
  offset%=intervals.size();
  while(times.back()<duration) {
    const auto i=(offset+times.size()-1)%intervals.size();
    times.push_back(times.back()+intervals[i]);
  }
  return times;
}
}
