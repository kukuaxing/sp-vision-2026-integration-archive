#pragma once
#include <cstdint>

// Passive recorder only. Callers serialize task/ISR access; no motor command
// or firing permission is produced here. Times use the wrapping MCU us clock.
struct FeedDiagnostics {
  uint16_t request_seq=0, issue_seq=0, motion_seq=0, encoder=0;
  uint32_t request_us=0, issue_us=0, motion_us=0, feedback_us=0;
  uint32_t issue_count=0, motion_count=0;
  int16_t rpm=0, baseline_encoder=0, direction=-1;
  bool request_seen=false, issued=false, motion_seen=false, feedback_seen=false;
  bool automatic=false, pending=false, already_moving=false;

  void Request(uint16_t seq, uint32_t us) {
    request_seq=seq; request_us=us; request_seen=true;
  }
  void Issue(uint16_t seq, uint32_t us, bool auto_request, int16_t dir) {
    issue_seq=seq; issue_us=us; automatic=auto_request; issued=true; ++issue_count;
    direction=dir; baseline_encoder=encoder;
    const bool fresh=feedback_seen && uint32_t(us-feedback_us)<=100000u;
    already_moving=fresh && (rpm>=100 || rpm<=-100);
    pending=fresh && !already_moving && direction!=0;
  }
  void Feedback(uint16_t angle, int16_t speed, uint32_t us) {
    if (pending && (uint32_t(us-feedback_us)>100000u ||
                    uint32_t(us-issue_us)>2000000u)) pending=false;
    encoder=angle; rpm=speed; feedback_us=us; feedback_seen=true;
    int32_t delta=int32_t(angle)-baseline_encoder;
    if (delta>4096) delta-=8192;
    if (delta<-4096) delta+=8192;
    // Confirm intended movement with both velocity and encoder displacement.
    // This is the first qualifying CAN sample, not muzzle exit or motor torque onset.
    if (pending && int32_t(speed)*direction>=100 && delta*direction>=8) {
      motion_seq=issue_seq; motion_us=us; ++motion_count; motion_seen=true; pending=false;
    }
  }
  uint8_t Flags(uint32_t us) const {
    return (feedback_seen && uint32_t(us-feedback_us)<=100000u ? 1u:0u) |
      (request_seen?2u:0u) | (issued?4u:0u) | (motion_seen?8u:0u) |
      (automatic?16u:0u) | (pending?32u:0u) | (already_moving?64u:0u);
  }
};
