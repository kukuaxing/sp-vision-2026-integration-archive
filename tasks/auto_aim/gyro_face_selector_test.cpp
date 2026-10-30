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

  // A full face period preserves the folded send angle, but refers to an
  // adjacent physical model face. Check both rotation directions and all ids.
  for (double omega : {-4.56, 4.56}) {
    const double horizon = 1.2517, bearing = 0.23;
    for (int face = 0; face < 4; ++face) {
      const double phase = bearing - face * M_PI / 2.0 - omega * horizon + 0.01;
      const auto selected = Selector::face_at_hit_time(phase, bearing, omega, horizon);
      assert(selected.valid && selected.face_id == face);
      assert(std::abs(selected.alignment_rad - 0.01) < 1e-9);
      const auto next = Selector::face_at_hit_time(
        phase, bearing, omega, horizon + M_PI / (2.0 * std::abs(omega)));
      assert(next.valid && next.face_id == (face + (omega > 0.0 ? 3 : 1)) % 4);
    }
  }
  assert(!Selector::face_at_hit_time(0, 0, 4.56, -0.1).valid);
  assert(!Selector::face_at_hit_time(NAN, 0, 4.56, 1.0).valid);

  std::cout << "gyro_face_selector_test: PASS\n";
  return 0;
}
