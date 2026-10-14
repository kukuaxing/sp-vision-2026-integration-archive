#pragma once

// Raw sensor hand-eye observations need verified rigid two-axis response, not
// an already calibrated camera-support mounting transform. HI91 keeps its gate.
inline bool calibration_capture_allowed(bool rmv, bool hipnuc, bool mounted_verified,
                                        bool raw_rigid_response_verified) {
  if (rmv) return raw_rigid_response_verified;
  if (hipnuc) return mounted_verified;
  return true;
}
