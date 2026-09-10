#include "gyro_face_selector.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

int main()
{
  using Selector = auto_aim::GyroFaceSelector;
  Selector selector(15.0 * M_PI / 180.0);

  auto first = selector.select({{0, 0.03}, {1, 1.55}, {2, 3.12}, {3, -1.58}}, 0.0, 10);
  assert(first.valid && first.face_id == 0 && !first.changed);

  // Noise makes the adjacent face look marginally better, but the same event
  // retains the physical face that was already committed.
  auto held = selector.select({{0, 0.08}, {1, 0.07}}, 0.0, 10);
  assert(held.valid && held.face_id == 0 && !held.changed);

  auto next = selector.select({{0, 1.55}, {1, 0.02}}, 0.0, 11);
  assert(next.valid && next.face_id == 1 && next.changed);

  auto invalid = selector.select({{0, 0.40}, {1, -0.50}}, 0.0, 12);
  assert(!invalid.valid);

  selector.reset();
  auto reacquired = selector.select({{3, -0.01}}, 0.0, 20);
  assert(reacquired.valid && reacquired.face_id == 3 && !reacquired.changed);

  std::cout << "gyro_face_selector_test: PASS\n";
  return 0;
}
