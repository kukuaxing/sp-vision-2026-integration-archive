#pragma once
#include <string>
#include <vector>

namespace io {
// Algorithm inputs do not depend on SocketCAN or a physical control board.
enum ShootMode { left_shoot, right_shoot, both_shoot };
inline const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};
}
