#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "tasks/auto_aim/robust_range_filter.hpp"
#include "tasks/auto_aim/shot_delay_adapter.hpp"

// Immutable settings: YAML is the only parameter source. No device I/O here.
// Keep the original expressions/defaults; this is a structural extraction.
struct StandardSettings
{
  explicit StandardSettings(const YAML::Node & yaml, bool sentinel)
  : integration_yaml(yaml), sentinel_protocol(sentinel)
  {
    if (xuc_periodic_fire_enabled) {
      const int minimum_ms = yaml["sentinel_fire_min_interval_ms"]
        ? yaml["sentinel_fire_min_interval_ms"].as<int>() : 1000;
      const bool teammate_prediction = yaml["xuc_high_speed_strategy"] &&
        yaml["xuc_high_speed_strategy"].as<std::string>() == "teammate_prediction";
      if (!sentinel || !xuc_event_phase_window_only ||
          !std::isfinite(xuc_event_fixed_range_m) || xuc_event_fixed_range_m <= 0.0 ||
          !std::isfinite(xuc_event_fixed_omega_rad_s) || xuc_event_fixed_omega_rad_s <= 0.0 ||
          !std::isfinite(xuc_periodic_phase_offset_s) ||
          std::abs(xuc_periodic_phase_offset_s) > 2.0 ||
          xuc_periodic_every_n_faces < 1 || xuc_periodic_every_n_faces > 16 ||
          minimum_ms < 200 || minimum_ms > 5000 ||
          // Native prediction skips opportunities during the transport cooldown;
          // its cadence is not scheduled by this legacy periodic face divider.
          (!teammate_prediction &&
            xuc_periodic_every_n_faces * (M_PI/2.0) / xuc_event_fixed_omega_rad_s * 1000.0
              < minimum_ms + 5.0))
        throw std::invalid_argument("Periodic fire requires Sentinel fixed-centre phase mode, finite offset +/-2000ms, faces 1..16, and period >= fire minimum + 5ms");
    }
  }

  const YAML::Node integration_yaml;
  const bool sentinel_protocol;
  const bool competition_headless =
    integration_yaml["competition_headless"] &&
    integration_yaml["competition_headless"].as<bool>();
  const bool direct_yaw_guard_enabled =
    integration_yaml["xuc_direct_yaw_guard"] &&
    integration_yaml["xuc_direct_yaw_guard"].as<bool>();
  const bool prediction_lead_limit_configured =
    integration_yaml["xuc_max_prediction_lead_rad"].IsDefined();
  const double max_prediction_lead_rad =
    prediction_lead_limit_configured
      ? std::max(0.0, integration_yaml["xuc_max_prediction_lead_rad"].as<double>())
      : 0.0;
  const double direct_yaw_filter_tau_s =
    integration_yaml["xuc_direct_yaw_filter_tau_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_filter_tau_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_yaw_hold_s =
    integration_yaml["xuc_direct_yaw_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_hold_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_yaw_max_rate_rad_s =
    integration_yaml["xuc_direct_yaw_max_rate_dps"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_max_rate_dps"].as<double>()) * M_PI / 180.0
      : 0.0;
  const double final_yaw_command_max_rate_rad_s =
    integration_yaml["xuc_final_yaw_command_max_rate_dps"]
      ? std::max(
          0.0, integration_yaml["xuc_final_yaw_command_max_rate_dps"].as<double>()) *
          M_PI / 180.0
      : 0.0;
  // Keep the fixed camera/gimbal boresight zero separate from the empirical
  // impact correction.  The former centers the visual line of sight; the latter
  // deliberately leaves the armor off-centre so the bore, not the camera, points
  // at the desired impact point.
  const double direct_yaw_boresight_correction_rad =
    integration_yaml["xuc_direct_yaw_boresight_correction_deg"]
      ? std::clamp(
          integration_yaml["xuc_direct_yaw_boresight_correction_deg"].as<double>(),
          -5.0,
          5.0) * M_PI / 180.0
      : 0.0;
  const double direct_yaw_impact_correction_rad =
    integration_yaml["xuc_direct_yaw_impact_correction_deg"]
      ? std::clamp(
          integration_yaml["xuc_direct_yaw_impact_correction_deg"].as<double>(),
          -10.0,
          10.0) * M_PI / 180.0
      : 0.0;
  const double direct_yaw_command_correction_rad = std::clamp(
    direct_yaw_boresight_correction_rad + direct_yaw_impact_correction_rad,
    -10.0 * M_PI / 180.0,
    10.0 * M_PI / 180.0);
  // The lower yaw loop can retain a static error even though the absolute target
  // is received correctly (field logs showed about 1.2 deg).  Close that gap in
  // the one coordinate both packet directions actually share: raw lower-board
  // yaw.  This layer sits after target prediction, so it removes actuator lag and
  // bias without replacing the ESKF/phase lead used for moving targets.
  const bool yaw_tracking_pi_enabled =
    integration_yaml["xuc_yaw_tracking_pi_enabled"] &&
    integration_yaml["xuc_yaw_tracking_pi_enabled"].as<bool>();
  const double yaw_tracking_kp = integration_yaml["xuc_yaw_tracking_kp"]
    ? std::clamp(integration_yaml["xuc_yaw_tracking_kp"].as<double>(), 0.0, 2.0)
    : 0.0;
  const double yaw_tracking_ki_per_s = integration_yaml["xuc_yaw_tracking_ki_per_s"]
    ? std::clamp(integration_yaml["xuc_yaw_tracking_ki_per_s"].as<double>(), 0.0, 5.0)
    : 0.0;
  const double yaw_tracking_integral_limit_rad =
    (integration_yaml["xuc_yaw_tracking_integral_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_integral_limit_deg"].as<double>(),
          0.0,
          5.0)
      : 0.0) * M_PI / 180.0;
  const double yaw_tracking_output_limit_rad =
    (integration_yaml["xuc_yaw_tracking_output_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_output_limit_deg"].as<double>(),
          0.0,
          8.0)
      : 0.0) * M_PI / 180.0;
  const double yaw_tracking_integrate_error_limit_rad =
    (integration_yaml["xuc_yaw_tracking_integrate_error_limit_deg"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_integrate_error_limit_deg"].as<double>(),
          0.1,
          10.0)
      : 3.0) * M_PI / 180.0;
  const double yaw_tracking_arm_dwell_s =
    (integration_yaml["xuc_yaw_tracking_arm_dwell_ms"]
      ? std::clamp(
          integration_yaml["xuc_yaw_tracking_arm_dwell_ms"].as<double>(),
          0.0,
          2000.0)
      : 500.0) / 1000.0;
  const bool direct_pitch_guard_enabled =
    integration_yaml["xuc_direct_pitch_guard"] &&
    integration_yaml["xuc_direct_pitch_guard"].as<bool>();
  const double direct_pitch_filter_tau_s =
    integration_yaml["xuc_direct_pitch_filter_tau_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_filter_tau_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_pitch_hold_s =
    integration_yaml["xuc_direct_pitch_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_hold_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_pitch_max_rate_rad_s =
    integration_yaml["xuc_direct_pitch_max_rate_dps"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_max_rate_dps"].as<double>()) * M_PI / 180.0
      : 0.0;
  const double direct_pitch_error_gain =
    integration_yaml["xuc_direct_pitch_error_gain"]
      ? std::clamp(integration_yaml["xuc_direct_pitch_error_gain"].as<double>(), 0.0, 2.0)
      : 1.0;
  // The lower-board feedback field is temporarily used for feeder diagnostics,
  // so it must never be interpreted as projectile speed.  Keep trajectory timing
  // on an explicit, bounded configuration value instead.
  const double xuc_projectile_speed_mps = integration_yaml["xuc_projectile_speed_mps"]
    ? std::clamp(integration_yaml["xuc_projectile_speed_mps"].as<double>(), 1.0, 50.0)
    : 23.0;
  // The real-shot reference was recorded at a specific friction-wheel speed.
  // Keep that calibration speed independent from the current launch speed;
  // otherwise changing wheel RPM would silently re-fit the fixed boresight bias
  // and cancel most of the intended trajectory change.
  const double xuc_ballistic_reference_projectile_speed_mps =
    integration_yaml["xuc_ballistic_reference_projectile_speed_mps"]
      ? std::clamp(
          integration_yaml["xuc_ballistic_reference_projectile_speed_mps"].as<double>(),
          1.0,
          50.0)
      : xuc_projectile_speed_mps;
  const bool xuc_ballistic_pitch_enabled = integration_yaml["xuc_ballistic_pitch_enabled"] &&
    integration_yaml["xuc_ballistic_pitch_enabled"].as<bool>();
  const double xuc_camera_forward_of_muzzle_m =
    integration_yaml["xuc_camera_forward_of_muzzle_m"]
      ? std::max(0.0, integration_yaml["xuc_camera_forward_of_muzzle_m"].as<double>())
      : 0.0;
  const double xuc_camera_above_muzzle_m = integration_yaml["xuc_camera_above_muzzle_m"]
    ? std::max(0.0, integration_yaml["xuc_camera_above_muzzle_m"].as<double>())
    : 0.0;
  const double xuc_ballistic_reference_camera_range_m =
    integration_yaml["xuc_ballistic_reference_camera_range_m"]
      ? std::max(0.10, integration_yaml["xuc_ballistic_reference_camera_range_m"].as<double>())
      : 1.0;
  const double xuc_ballistic_reference_total_drop_m =
    integration_yaml["xuc_ballistic_reference_total_drop_m"]
      ? std::max(0.0, integration_yaml["xuc_ballistic_reference_total_drop_m"].as<double>())
      : 0.0;
  const double xuc_ballistic_max_compensation_rad =
    (integration_yaml["xuc_ballistic_max_compensation_deg"]
       ? std::clamp(
           integration_yaml["xuc_ballistic_max_compensation_deg"].as<double>(), 0.0, 25.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_ballistic_pitch_trim_rad =
    (integration_yaml["xuc_ballistic_pitch_trim_deg"]
       ? std::clamp(
           integration_yaml["xuc_ballistic_pitch_trim_deg"].as<double>(), -5.0, 5.0)
       : 0.0) * M_PI / 180.0;
  const auto_aim::RobustRangeFilter::Config robust_range_config = make_robust_range_config();
  const double direct_pitch_focal_y_px =
    integration_yaml["camera_matrix"] && integration_yaml["camera_matrix"].size() >= 5
      ? integration_yaml["camera_matrix"][4].as<double>()
      : 0.0;
  const double direct_yaw_focal_x_px =
    integration_yaml["camera_matrix"] && integration_yaml["camera_matrix"].size() >= 1
      ? integration_yaml["camera_matrix"][0].as<double>()
      : 0.0;
  const double xuc_pitch_sign_for_log = integration_yaml["xuc_pitch_sign"]
    ? integration_yaml["xuc_pitch_sign"].as<double>()
    : -1.0;
  const bool xuc_fire_guard_enabled = integration_yaml["xuc_fire_guard_enabled"] &&
    integration_yaml["xuc_fire_guard_enabled"].as<bool>();
  const double xuc_fire_stable_s = integration_yaml["xuc_fire_stable_ms"]
    ? std::max(0.0, integration_yaml["xuc_fire_stable_ms"].as<double>()) / 1000.0
    : 0.5;
  const double xuc_fire_yaw_tolerance_rad = integration_yaml["xuc_fire_yaw_tolerance_deg"]
    ? std::max(0.0, integration_yaml["xuc_fire_yaw_tolerance_deg"].as<double>()) * M_PI / 180.0
    : 1.0 * M_PI / 180.0;
  const double xuc_fire_pitch_tolerance_rad = integration_yaml["xuc_fire_pitch_tolerance_deg"]
    ? std::max(0.0, integration_yaml["xuc_fire_pitch_tolerance_deg"].as<double>()) * M_PI / 180.0
    : 1.0 * M_PI / 180.0;
  const bool xuc_gyro_center_follow_enabled =
    integration_yaml["xuc_gyro_center_follow_enabled"] &&
    integration_yaml["xuc_gyro_center_follow_enabled"].as<bool>();
  const double xuc_gyro_omega_threshold_rad_s =
    integration_yaml["xuc_gyro_omega_threshold_rad_s"]
      ? std::max(0.0, integration_yaml["xuc_gyro_omega_threshold_rad_s"].as<double>())
      : 1.0;
  const double xuc_vehicle_model_enter_rad_s =
    integration_yaml["xuc_vehicle_model_enter_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_vehicle_model_enter_rad_s"].as<double>(), 0.0, 10.0)
      : xuc_gyro_omega_threshold_rad_s;
  const double xuc_vehicle_model_exit_rad_s =
    integration_yaml["xuc_vehicle_model_exit_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_vehicle_model_exit_rad_s"].as<double>(), 0.0,
          xuc_vehicle_model_enter_rad_s)
      : std::min(0.5, xuc_vehicle_model_enter_rad_s);
  const double xuc_high_speed_enter_rad_s =
    integration_yaml["xuc_high_speed_enter_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_high_speed_enter_rad_s"].as<double>(), 0.1, 20.0)
      : 3.2;
  const double xuc_high_speed_exit_rad_s =
    integration_yaml["xuc_high_speed_exit_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_high_speed_exit_rad_s"].as<double>(), 0.0,
          xuc_high_speed_enter_rad_s)
      : std::min(2.5, xuc_high_speed_enter_rad_s);
  const double xuc_rotation_mode_dwell_s =
    integration_yaml["xuc_rotation_mode_dwell_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_mode_dwell_ms"].as<double>(), 0.0, 1000.0) /
          1000.0
      : 0.15;
  const double xuc_rotation_tracker_dropout_hold_s =
    integration_yaml["xuc_rotation_tracker_dropout_hold_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_tracker_dropout_hold_ms"].as<double>(),
          0.0, 1000.0) / 1000.0
      : 0.0;
  const double xuc_rotation_rate_filter_tau_s =
    integration_yaml["xuc_rotation_rate_filter_tau_ms"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_filter_tau_ms"].as<double>(),
          0.0, 1000.0) / 1000.0
      : 0.0;
  const double xuc_rotation_rate_disagreement_rad_s =
    integration_yaml["xuc_rotation_rate_disagreement_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_disagreement_rad_s"].as<double>(),
          0.1, 10.0)
      : 1.2;
  const double xuc_rotation_speed_step_reset_rad_s =
    integration_yaml["xuc_rotation_speed_step_reset_rad_s"]
      ? std::clamp(
          integration_yaml["xuc_rotation_speed_step_reset_rad_s"].as<double>(),
          0.1, 10.0)
      : 1.2;
  const int xuc_rotation_rate_agreement_updates =
    integration_yaml["xuc_rotation_rate_agreement_updates"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_agreement_updates"].as<int>(), 1, 60)
      : 2;
  const int xuc_rotation_rate_disagreement_updates =
    integration_yaml["xuc_rotation_rate_disagreement_updates"]
      ? std::clamp(
          integration_yaml["xuc_rotation_rate_disagreement_updates"].as<int>(), 2, 120)
      : 3;
  const double xuc_gyro_center_hold_s =
    integration_yaml["xuc_gyro_center_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_gyro_center_hold_ms"].as<double>()) / 1000.0
      : 0.6;
  const bool xuc_fire_require_tracker_converged =
    integration_yaml["xuc_fire_require_tracker_converged"] &&
    integration_yaml["xuc_fire_require_tracker_converged"].as<bool>();
  const bool xuc_one_face_once_fire =
    integration_yaml["xuc_one_face_once_fire"] &&
    integration_yaml["xuc_one_face_once_fire"].as<bool>();
  const double xuc_one_face_limit_rad =
    (integration_yaml["xuc_one_face_limit_deg"]
       ? std::clamp(integration_yaml["xuc_one_face_limit_deg"].as<double>(), 1.0, 45.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_one_face_rearm_rad =
    (integration_yaml["xuc_one_face_rearm_deg"]
       ? std::clamp(integration_yaml["xuc_one_face_rearm_deg"].as<double>(), 5.0, 90.0)
       : 34.0) * M_PI / 180.0;
  const double xuc_gyro_fire_stable_s =
    integration_yaml["xuc_gyro_fire_stable_ms"]
      ? std::max(0.0, integration_yaml["xuc_gyro_fire_stable_ms"].as<double>()) / 1000.0
      : 0.03;
  // A stationary target is still qualified by the proven image-space gate.
  // In gyro mode, however, the visible plate is moving while the gimbal follows
  // the vehicle centre. Qualify the armor selected by Aimer at projectile impact
  // time instead of reusing the current-frame pixel error as the precise trigger.
  const bool xuc_gyro_predicted_fire_enabled =
    integration_yaml["xuc_gyro_predicted_fire_enabled"] &&
    integration_yaml["xuc_gyro_predicted_fire_enabled"].as<bool>();
  // A native yaw-error fallback was useful while bringing up the event clock,
  // but it has no phase/cycle information.  Let field configs fail closed while
  // the PLL is unlocked, without changing legacy configs that omit this key.
  const bool xuc_gyro_require_event_lock_for_fire =
    integration_yaml["xuc_gyro_require_event_lock_for_fire"] &&
    integration_yaml["xuc_gyro_require_event_lock_for_fire"].as<bool>();
  const double xuc_gyro_predicted_yaw_tolerance_rad =
    (integration_yaml["xuc_gyro_predicted_yaw_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_predicted_yaw_tolerance_deg"].as<double>(),
           0.1,
           10.0)
       : 1.0) * M_PI / 180.0;
  const double xuc_gyro_predicted_yaw_trim_rad =
    (integration_yaml["xuc_gyro_predicted_yaw_trim_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_predicted_yaw_trim_deg"].as<double>(), -5.0, 5.0)
       : 0.0) * M_PI / 180.0;
  const double xuc_gyro_visibility_yaw_tolerance_rad =
    (integration_yaml["xuc_gyro_visibility_yaw_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_visibility_yaw_tolerance_deg"].as<double>(),
           0.5,
           20.0)
       : 6.0) * M_PI / 180.0;
  const int xuc_gyro_fire_min_consecutive_frames =
    integration_yaml["xuc_gyro_fire_min_consecutive_frames"]
      ? std::clamp(
          integration_yaml["xuc_gyro_fire_min_consecutive_frames"].as<int>(), 1, 10)
      : 2;
  // Predictive fire uses one-frame pulses. Pending-shot and live feeder feedback
  // prevent overlapping requests; an optional post-feed hold remains available
  // for lower boards that require it, while zero enables feedback-driven repeat.
  const double xuc_gyro_post_feed_recovery_s =
    integration_yaml["xuc_gyro_post_feed_recovery_ms"]
      ? std::clamp(
          integration_yaml["xuc_gyro_post_feed_recovery_ms"].as<double>(),
          0.0,
          2000.0) /
          1000.0
      : 0.6;
  // Event-clock fire timing. The calibrated nominal remains the fallback;
  // receive-thread feeder timing may add a small, bounded runtime correction.
  // Positive fixed range enables the explicit stationary-centre bench policy.
  // Zero preserves the ordinary measured-range event clock.
  const double xuc_event_fixed_range_m = integration_yaml["xuc_event_fixed_range_m"]
    ? std::clamp(integration_yaml["xuc_event_fixed_range_m"].as<double>(), 0.0, 20.0) : 0.0;
  const double xuc_event_fixed_omega_rad_s = integration_yaml["xuc_event_fixed_omega_rad_s"]
    ? std::clamp(integration_yaml["xuc_event_fixed_omega_rad_s"].as<double>(), 0.0, 8.0) : 0.0;
  const double xuc_event_command_phase_tolerance_rad =
    (integration_yaml["xuc_event_command_phase_tolerance_deg"]
      ? std::clamp(integration_yaml["xuc_event_command_phase_tolerance_deg"].as<double>(), 0.2, 5.0)
      : 2.0) * M_PI / 180.0;
  const bool xuc_event_phase_window_only = integration_yaml["xuc_event_phase_window_only"] &&
    integration_yaml["xuc_event_phase_window_only"].as<bool>();
  const bool xuc_event_phase_after_only = integration_yaml["xuc_event_phase_after_only"] &&
    integration_yaml["xuc_event_phase_after_only"].as<bool>();
  const bool xuc_periodic_fire_enabled = integration_yaml["xuc_periodic_fire_enabled"] &&
    integration_yaml["xuc_periodic_fire_enabled"].as<bool>();
  const double xuc_periodic_phase_offset_s = integration_yaml["xuc_periodic_phase_offset_ms"]
    ? integration_yaml["xuc_periodic_phase_offset_ms"].as<double>() / 1000.0 : 0.0;
  const int xuc_periodic_every_n_faces = integration_yaml["xuc_periodic_every_n_faces"]
    ? integration_yaml["xuc_periodic_every_n_faces"].as<int>() : 1;
  const double xuc_event_fire_delay_s =
    integration_yaml["xuc_event_fire_delay_s"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_s"].as<double>(), 0.0, 2.0)
      : 0.10;
  const double xuc_event_fire_delay_min_s =
    integration_yaml["xuc_event_fire_delay_min_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_min_ms"].as<double>(), 0.0, 2000.0) /
          1000.0
      : std::max(0.0, xuc_event_fire_delay_s - 0.020);
  const double xuc_event_fire_delay_max_s =
    integration_yaml["xuc_event_fire_delay_max_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_fire_delay_max_ms"].as<double>(), 0.0, 2000.0) /
          1000.0
      : std::min(2.0, xuc_event_fire_delay_s + 0.020);
  const auto_aim::ShotDelayAdapter::Config shot_delay_config = make_shot_delay_config();
  const double xuc_event_flight_time_uncertainty_s =
    integration_yaml["xuc_event_flight_time_uncertainty_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_flight_time_uncertainty_ms"].as<double>(),
          0.0,
          100.0) /
          1000.0
      : 0.005;
  // If the precise due time lies shortly after this frame, wait until that
  // steady-clock instant and send the one-frame pulse there.  This removes the
  // 16 ms quantization of a ~60 FPS vision loop without delaying ordinary
  // tracking frames.
  const double xuc_event_trigger_window_s =
    integration_yaml["xuc_event_trigger_window_ms"]
      ? std::max(0.0, integration_yaml["xuc_event_trigger_window_ms"].as<double>()) /
          1000.0
      : 0.025;
  const double xuc_event_send_late_tolerance_s =
    integration_yaml["xuc_event_send_late_tolerance_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_send_late_tolerance_ms"].as<double>(),
          0.0,
          10.0) /
          1000.0
      : 0.002;
  const double xuc_event_crossing_capture_rad =
    (integration_yaml["xuc_event_crossing_capture_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_crossing_capture_deg"].as<double>(), 5.0, 45.0)
       : 20.0) * M_PI / 180.0;
  const double xuc_event_crossing_rearm_rad =
    (integration_yaml["xuc_event_crossing_rearm_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_crossing_rearm_deg"].as<double>(), 2.0, 35.0)
       : 12.0) * M_PI / 180.0;
  const double xuc_event_min_crossing_separation_ratio =
    integration_yaml["xuc_event_min_crossing_separation_ratio"]
      ? std::clamp(
          integration_yaml["xuc_event_min_crossing_separation_ratio"].as<double>(),
          0.20,
          0.90)
      : 0.55;
  const double xuc_event_period_min_s =
    integration_yaml["xuc_event_period_min_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_period_min_ms"].as<double>(), 30.0, 2000.0) /
          1000.0
      : 0.12;
  const double xuc_event_period_max_s =
    integration_yaml["xuc_event_period_max_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_period_max_ms"].as<double>(), 50.0, 5000.0) /
          1000.0
      : 1.20;
  const double xuc_event_period_ema_alpha =
    integration_yaml["xuc_event_period_ema_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_period_ema_alpha"].as<double>(), 0.01, 1.0)
      : 0.25;
  const double xuc_event_max_period_relative_error =
    integration_yaml["xuc_event_max_period_relative_error"]
      ? std::clamp(
          integration_yaml["xuc_event_max_period_relative_error"].as<double>(),
          0.05,
          0.50)
      : 0.22;
  const double xuc_event_max_period_step_ratio =
    integration_yaml["xuc_event_max_period_step_ratio"]
      ? std::clamp(
          integration_yaml["xuc_event_max_period_step_ratio"].as<double>(),
          0.001,
          0.20)
      : 0.025;
  const double xuc_event_omega_period_weight =
    integration_yaml["xuc_event_omega_period_weight"]
      ? std::clamp(
          integration_yaml["xuc_event_omega_period_weight"].as<double>(),
          0.0,
          1.0)
      : 0.15;
  const double xuc_event_period_residual_alpha =
    integration_yaml["xuc_event_period_residual_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_period_residual_alpha"].as<double>(),
          0.0,
          0.5)
      : 0.08;
  const double xuc_event_phase_correction_alpha =
    integration_yaml["xuc_event_phase_correction_alpha"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_correction_alpha"].as<double>(),
          0.0,
          1.0)
      : 0.45;
  const double xuc_event_max_phase_correction_s =
    integration_yaml["xuc_event_max_phase_correction_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_max_phase_correction_ms"].as<double>(),
          0.0,
          20.0) /
          1000.0
      : 0.015;
  const double xuc_event_max_phase_residual_s =
    integration_yaml["xuc_event_max_phase_residual_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_max_phase_residual_ms"].as<double>(),
          5.0,
          150.0) /
          1000.0
      : 0.090;
  const int xuc_event_period_window_size =
    integration_yaml["xuc_event_period_window_size"]
      ? std::clamp(
          integration_yaml["xuc_event_period_window_size"].as<int>(), 3, 15)
      : 7;
  const double xuc_event_phase_uncertainty_floor_s =
    integration_yaml["xuc_event_phase_uncertainty_floor_ms"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_uncertainty_floor_ms"].as<double>(),
          0.0,
          50.0) /
          1000.0
      : 0.002;
  const int xuc_event_phase_residual_window_size =
    integration_yaml["xuc_event_phase_residual_window_size"]
      ? std::clamp(
          integration_yaml["xuc_event_phase_residual_window_size"].as<int>(), 3, 15)
      : 5;
  const int xuc_event_min_lock_updates =
    integration_yaml["xuc_event_min_lock_updates"]
      ? std::clamp(
          integration_yaml["xuc_event_min_lock_updates"].as<int>(), 2, 12)
      : 4;
  const int xuc_event_relock_rejections =
    integration_yaml["xuc_event_relock_rejections"]
      ? std::clamp(
          integration_yaml["xuc_event_relock_rejections"].as<int>(), 2, 12)
      : 3;
  const double xuc_event_armor_width_m =
    integration_yaml["xuc_event_armor_width_m"]
      ? std::clamp(
          integration_yaml["xuc_event_armor_width_m"].as<double>(), 0.05, 0.40)
      : 0.135;
  const double xuc_event_armor_edge_margin_m =
    integration_yaml["xuc_event_armor_edge_margin_m"]
      ? std::clamp(
          integration_yaml["xuc_event_armor_edge_margin_m"].as<double>(), 0.0, 0.10)
      : 0.005;
  const double xuc_event_model_phase_uncertainty_rad =
    (integration_yaml["xuc_event_model_phase_uncertainty_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_model_phase_uncertainty_deg"].as<double>(),
           0.0,
           20.0)
       : 1.0) * M_PI / 180.0;
  // A small, explicit grace angle relaxes the conservative worst-case budget
  // without bypassing phase lock or gimbal tracking.
  const double xuc_event_phase_grace_rad =
    (integration_yaml["xuc_event_phase_grace_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_phase_grace_deg"].as<double>(), 0.0, 4.0)
       : 0.0) * M_PI / 180.0;
  // The crossing PLL observes the physical face-center event directly.  The
  // EKF future-face prediction is useful telemetry, but its phase ripples at
  // armor hand-over can be made a soft check for higher fire availability.
  const bool xuc_event_require_face_consistency =
    !integration_yaml["xuc_event_require_face_consistency"] ||
    integration_yaml["xuc_event_require_face_consistency"].as<bool>();
  const double xuc_event_face_max_alignment_rad =
    (integration_yaml["xuc_event_face_max_alignment_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_face_max_alignment_deg"].as<double>(),
           1.0,
           45.0)
       : 15.0) * M_PI / 180.0;
  const double xuc_event_gimbal_tracking_tolerance_rad =
    (integration_yaml["xuc_event_gimbal_tracking_tolerance_deg"]
       ? std::clamp(
           integration_yaml["xuc_event_gimbal_tracking_tolerance_deg"].as<double>(),
           0.2,
           10.0)
       : 2.5) * M_PI / 180.0;
  const double xuc_shot_feedback_timeout_s =
    integration_yaml["xuc_shot_feedback_timeout_ms"]
      ? std::clamp(
          integration_yaml["xuc_shot_feedback_timeout_ms"].as<double>(), 100.0, 1000.0) /
          1000.0
      : 0.35;
  // A face switch is approximately 90 degrees on a four-armor target. The
  // same face moves only about five degrees per frame at 60 FPS and 5 rad/s.
  // Reject a two-frame qualification that silently crosses a face switch.
  const double xuc_gyro_candidate_max_phase_step_rad =
    (integration_yaml["xuc_gyro_candidate_max_phase_step_deg"]
       ? std::clamp(
           integration_yaml["xuc_gyro_candidate_max_phase_step_deg"].as<double>(),
           5.0,
           45.0)
       : 20.0) * M_PI / 180.0;

private:
  auto_aim::RobustRangeFilter::Config make_robust_range_config() const
  {
    auto_aim::RobustRangeFilter::Config robust_range_config;
    robust_range_config.time_constant_s =
      integration_yaml["xuc_range_filter_tau_ms"]
        ? std::clamp(
            integration_yaml["xuc_range_filter_tau_ms"].as<double>(), 0.0, 500.0) /
            1000.0
        : 0.08;
    robust_range_config.max_rate_mps =
      integration_yaml["xuc_range_filter_max_rate_mps"]
        ? std::clamp(
            integration_yaml["xuc_range_filter_max_rate_mps"].as<double>(), 0.1, 20.0)
        : 6.0;
    robust_range_config.raw_blend =
      integration_yaml["xuc_range_filter_raw_blend"]
        ? std::clamp(
            integration_yaml["xuc_range_filter_raw_blend"].as<double>(), 0.0, 0.5)
        : 0.05;
    robust_range_config.max_raw_model_residual_m =
      integration_yaml["xuc_range_filter_max_raw_residual_m"]
        ? std::clamp(
            integration_yaml["xuc_range_filter_max_raw_residual_m"].as<double>(),
            0.05,
            2.0)
        : 0.40;
    return robust_range_config;
  }

  auto_aim::ShotDelayAdapter::Config make_shot_delay_config() const
  {
    auto_aim::ShotDelayAdapter::Config shot_delay_config;
    shot_delay_config.enabled =
      integration_yaml["xuc_shot_delay_adaptation_enabled"] &&
      integration_yaml["xuc_shot_delay_adaptation_enabled"].as<bool>();
    shot_delay_config.nominal_delay_s = xuc_event_fire_delay_s;
    shot_delay_config.min_delay_s = std::min(
      xuc_event_fire_delay_min_s, xuc_event_fire_delay_max_s);
    shot_delay_config.max_delay_s = std::max(
      xuc_event_fire_delay_min_s, xuc_event_fire_delay_max_s);
    shot_delay_config.reference_feedback_s =
      integration_yaml["xuc_shot_delay_reference_ms"]
        ? std::max(0.0, integration_yaml["xuc_shot_delay_reference_ms"].as<double>()) /
            1000.0
        : 0.0;
    shot_delay_config.sample_min_s =
      (integration_yaml["xuc_shot_delay_sample_min_ms"]
         ? std::clamp(
             integration_yaml["xuc_shot_delay_sample_min_ms"].as<double>(), 0.0, 200.0)
         : 5.0) / 1000.0;
    shot_delay_config.sample_max_s =
      (integration_yaml["xuc_shot_delay_sample_max_ms"]
         ? std::clamp(
             integration_yaml["xuc_shot_delay_sample_max_ms"].as<double>(), 5.0, 500.0)
         : 100.0) / 1000.0;
    shot_delay_config.window_size = static_cast<std::size_t>(
      integration_yaml["xuc_shot_delay_window"]
        ? std::clamp(integration_yaml["xuc_shot_delay_window"].as<int>(), 3, 31)
        : 9);
    shot_delay_config.min_samples = static_cast<std::size_t>(
      integration_yaml["xuc_shot_delay_min_samples"]
        ? std::clamp(integration_yaml["xuc_shot_delay_min_samples"].as<int>(), 3, 31)
        : 5);
    shot_delay_config.ewma_alpha =
      integration_yaml["xuc_shot_delay_ewma_alpha"]
        ? std::clamp(
            integration_yaml["xuc_shot_delay_ewma_alpha"].as<double>(), 0.01, 1.0)
        : 0.35;
    shot_delay_config.adaptation_gain =
      integration_yaml["xuc_shot_delay_adaptation_gain"]
        ? std::clamp(
            integration_yaml["xuc_shot_delay_adaptation_gain"].as<double>(), 0.0, 1.0)
        : 1.0;
    shot_delay_config.max_step_s =
      (integration_yaml["xuc_shot_delay_max_step_ms"]
         ? std::clamp(
             integration_yaml["xuc_shot_delay_max_step_ms"].as<double>(), 0.0, 10.0)
         : 1.5) / 1000.0;
    shot_delay_config.outlier_sigma =
      integration_yaml["xuc_shot_delay_outlier_sigma"]
        ? std::clamp(
            integration_yaml["xuc_shot_delay_outlier_sigma"].as<double>(), 1.0, 10.0)
        : 3.5;
    shot_delay_config.outlier_floor_s =
      (integration_yaml["xuc_shot_delay_outlier_floor_ms"]
         ? std::clamp(
             integration_yaml["xuc_shot_delay_outlier_floor_ms"].as<double>(), 0.0, 50.0)
         : 4.0) / 1000.0;
    shot_delay_config.min_prediction_half_width_s =
      (integration_yaml["xuc_shot_delay_95pi_min_half_ms"]
         ? std::clamp(
             integration_yaml["xuc_shot_delay_95pi_min_half_ms"].as<double>(), 1.0, 30.0)
         : 8.0) / 1000.0;
    return shot_delay_config;
  }
};
