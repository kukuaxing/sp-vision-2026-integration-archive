#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
  bool armor_detected = false;  // Current camera frame, after enemy-color filtering; not a predicted track.
};

}  // namespace io

#endif  // IO__COMMAND_HPP
