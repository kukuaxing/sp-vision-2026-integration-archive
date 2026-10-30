// Real detector-observation -> Solver -> Tracker -> Target path. No devices,
// model inference or mocked matching/filter implementation are involved.
#include "tasks/auto_aim/tracker.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include "tasks/auto_aim/rotation_model_manager.hpp"

namespace
{
struct Fixture
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("sp_tracker_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  Fixture()
  {
    std::ofstream file(path);
    file << "R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n"
         << "R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
         << "t_camera2gimbal: [0,0,0]\n"
         << "camera_matrix: [1000,0,640,0,1000,512,0,0,1]\n"
         << "distort_coeffs: [0,0,0,0,0]\n"
         << "enemy_color: blue\nmin_detect_count: 5\nmin_confidence: 0.1\n"
         << "max_temp_lost_count: 15\noutpost_max_temp_lost_count: 75\n"
         << "tracker_target_switch_min_frames: 3\n"
         << "center_process_adaptive_enabled: true\n"
         << "tracker_batch_min_omega_rad_s: 1.5\n"
         << "yaw_eskf_max_trusted_abs_rate_rad_s: 8\n"
         << "yaw_eskf_max_trusted_abs_acceleration_rad_s2: 30\n";
  }
  ~Fixture() { std::filesystem::remove(path); }
};

auto_aim::Armor observation(
  auto_aim::Solver & solver, const Eigen::Vector3d & position,
  int class_id = 11, std::mt19937 * random = nullptr, double sigma = 0.0, double yaw = 0.0)
{
  auto points = solver.reproject_armor(position, yaw, auto_aim::small, auto_aim::four);
  if (random) {
    std::normal_distribution<double> noise(0.0, sigma);
    for (auto & p : points) {
      p.x += static_cast<float>(noise(*random));
      p.y += static_cast<float>(noise(*random));
    }
  }
  auto_aim::Armor armor(
    class_id, .95F, cv::boundingRect(points), points, auto_aim::YOLOVersion::YOLO26);
  armor.center_norm = {armor.center.x / 1280.F, armor.center.y / 1024.F};
  return armor;
}
}

int main(int argc, char ** argv)
{
  if (argc == 2) {
    using namespace auto_aim;
    nlohmann::json frames; std::ifstream(argv[1]) >> frames;
    Solver solver("configs/standard.yaml"); Tracker tracker("configs/standard.yaml", solver);
    int rebuilt = 0, tracking = 0, trusted = 0, updated = 0, ids = 0;
    int rate_alias_resets = 0;
    std::uint64_t previous_id = 0;
    nlohmann::json model_samples = nlohmann::json::array();
    int consecutive_trusted = 0, longest_trusted = 0, whole_trusted = 0, vehicle_fresh = 0;
    const auto config = YAML::LoadFile("configs/standard.yaml");
    RotationModelManager::Config selection;
    selection.vehicle_enter_rad_s = config["xuc_vehicle_model_enter_rad_s"].as<double>();
    selection.vehicle_exit_rad_s = config["xuc_vehicle_model_exit_rad_s"].as<double>();
    selection.high_speed_enter_rad_s = config["xuc_high_speed_enter_rad_s"].as<double>();
    selection.high_speed_exit_rad_s = config["xuc_high_speed_exit_rad_s"].as<double>();
    selection.mode_dwell_s = config["xuc_rotation_mode_dwell_ms"].as<double>() / 1000.0;
    selection.tracker_dropout_hold_s = config["xuc_rotation_tracker_dropout_hold_ms"].as<double>() / 1000.0;
    selection.rate_filter_tau_s = config["xuc_rotation_rate_filter_tau_ms"].as<double>() / 1000.0;
    RotationModelManager selector(selection);
    for (const auto & frame : frames) {
      const auto rpy = frame["rpy"].get<std::vector<double>>();
      const Eigen::Quaterniond q(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));
      solver.set_R_gimbal2world_absolute_imu(q);
      std::list<Armor> armors;
      if (frame["pose_valid"].get<bool>()) for (const auto & detection : frame["detections"]) {
        int id = -1;
        for (std::size_t i = 0; i < yolo26_armor_properties.size(); ++i) {
          const auto [color, name, type] = yolo26_armor_properties[i];
          if (color == detection["color"].get<int>() && name == detection["name"].get<int>() &&
              type == detection["type"].get<int>()) { id = static_cast<int>(i); break; }
        }
        assert(id >= 0);
        std::vector<cv::Point2f> points;
        for (const auto & point : detection["corners_px"])
          points.emplace_back(point[0].get<float>(), point[1].get<float>());
        armors.emplace_back(id, detection["confidence"].get<float>(), cv::boundingRect(points),
          points, YOLOVersion::YOLO26);
        auto & armor = armors.back();
        armor.keypoint_confidence = detection["keypoint_confidence"].get<double>();
        armor.pose_observation_valid = detection["physical_corners_valid"].get<bool>();
        armor.center_norm = {armor.center.x / 1440.F, armor.center.y / 1080.F};
      }
      const auto time = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(
        static_cast<std::int64_t>(frame["t"].get<double>() * 1e9)));
      auto targets = tracker.track(armors, time);
      if (tracker.diagnostics().rate_alias_reset) {
        ++rate_alias_resets;
        assert(targets.empty());
        assert(!tracker.measurement_updated());
        assert(!tracker.identity_confirmed());
      }
      rebuilt += tracker.diagnostics().rebuilt;
      tracking += tracker.state() == "tracking";
      updated += tracker.measurement_updated();
      trusted += !targets.empty() && targets.front().yaw_eskf_trusted();
      const bool use_whole = !targets.empty() && targets.front().whole_rotation_trusted() &&
        std::abs(targets.front().ekf().x[7]) < selection.high_speed_enter_rad_s &&
        selector.mode() != RotationModelManager::Mode::high_speed_phase;
      whole_trusted += use_whole;
      const auto selected = selector.update(frame["t"].get<double>(),
        !targets.empty() && targets.front().convergened() &&
          (use_whole || targets.front().yaw_eskf_trusted()),
        targets.empty() ? 0.0 : use_whole ? targets.front().ekf().x[7] : targets.front().ekf_x()[7]);
      vehicle_fresh += selected.mode == RotationModelManager::Mode::vehicle && selected.tracker_trusted &&
        tracker.state() == "tracking" && tracker.measurement_updated();
      consecutive_trusted = !targets.empty() && targets.front().yaw_eskf_trusted() ? consecutive_trusted + 1 : 0;
      longest_trusted = std::max(longest_trusted, consecutive_trusted);
      if (!targets.empty() && frame["sequence"].get<int>() % 10 == 0) {
        const auto & model = targets.front(); const auto & filter = model.ekf();
        model_samples.push_back({{"frame",frame["sequence"]}, {"tracking",tracker.state() == "tracking"},
          {"fresh",tracker.measurement_updated()}, {"jumped",model.jumped},
          {"phase_variance",filter.P(6,6)}, {"rate_variance",filter.P(7,7)},
          {"center_x_variance",filter.P(0,0)}, {"center_y_variance",filter.P(2,2)},
          {"radius_variance",filter.P(8,8)}, {"omega",filter.x[7]},
          {"federated_trusted",model.yaw_eskf_trusted()}, {"nis",filter.last_nis}});
      }
      if (tracker.track_id() != 0 && tracker.track_id() != previous_id) {
        ++ids; previous_id = tracker.track_id();
      }
    }
    std::ofstream("logs/audit/rotation-model-confidence.json") << nlohmann::json({
      {"samples", model_samples}, {"longest_trusted_frames",longest_trusted}}).dump(2);
    std::cout << "FIELD_REPLAY " << nlohmann::json({{"frames", frames.size()}, {"rebuilt", rebuilt},
      {"tracking", tracking}, {"updated", updated}, {"yaw_trusted", trusted}, {"whole_trusted", whole_trusted}, {"vehicle_fresh", vehicle_fresh}, {"tracks", ids}, {"rate_alias_resets",rate_alias_resets}}).dump() << '\n';
    return 0;
  }
  using namespace auto_aim;
  Fixture fixture;
  Solver solver(fixture.path.string());
  const Eigen::Vector3d a(3.0, 0.0, 0.0);
  const Eigen::Vector3d b(3.0, 0.9, 0.0);
  const auto start = std::chrono::steady_clock::now();
  auto timestamp = [&](int frame) { return start + std::chrono::milliseconds(20 * frame); };

  // Initial acquisition must prefer the image center, irrespective of the
  // observation list order. The earlier score preferred the top-left origin.
  {
    Tracker tracker(fixture.path.string(), solver);
    const Eigen::Vector3d top_left(3.0, 0.45, 0.45);
    std::list<Armor> armors{observation(solver, top_left), observation(solver, a)};
    const auto targets = tracker.track(armors, timestamp(0));
    assert(targets.size() == 1);
    assert((targets.front().armor_xyza_list()[0].head<3>() - a).norm() < .01);
  }

  // Display-only network keypoints after a failed physical-bar extraction
  // cannot initialize a target or refresh an established geometric track.
  {
    Tracker tracker(fixture.path.string(), solver);
    auto invalid = observation(solver, a);
    invalid.pose_observation_valid = false;
    std::list<Armor> armors{invalid};
    assert(tracker.track(armors, timestamp(0)).empty());
    assert(tracker.state() == "lost" && tracker.track_id() == 0);
    for (int frame = 1; frame <= 10; ++frame) {
      armors = {observation(solver, a)};
      assert(tracker.track(armors, timestamp(frame)).size() == 1);
    }
    const auto id = tracker.track_id();
    for (int frame = 11; frame <= 13; ++frame) {
      armors = {invalid};
      assert(tracker.track(armors, timestamp(frame)).size() == 1);
      assert(!tracker.measurement_updated() && tracker.state() == "temp_lost");
      assert(tracker.track_id() == id && !armors.front().pose_valid);
    }
    armors = {observation(solver, a)};
    tracker.track(armors, timestamp(14));
    assert(tracker.measurement_updated() && tracker.track_id() == id);
    assert(tracker.state() == "tracking");
  }

  // Single-frame digit/size errors must not break an already matched image
  // object. Geometry stays identical; class 8 means big blue_1 in YOLO26.
  {
    Tracker tracker(fixture.path.string(), solver);
    std::uint64_t id = 0;
    for (int frame = 0; frame < 100; ++frame) {
      const int cls = frame < 10 || frame % 2 ? 11 : 8;
      std::list<Armor> armors{observation(solver, a, cls)};
      const auto targets = tracker.track(armors, timestamp(frame));
      assert(targets.size() == 1 && tracker.measurement_updated());
      if (frame == 0) id = tracker.track_id();
      assert(id != 0 && tracker.track_id() == id);
      assert(armors.front().pose_valid && armors.front().name == four && armors.front().type == small);
      if (frame >= 4) assert(tracker.state() == "tracking");
    }
    assert(tracker.identity_confirmed());
  }

  // Sustained 4-small -> 1-big class confusion on identical physical corners
  // reproduced 76 same-track rebuilds in the field. Repeated votes cannot
  // override worse size geometry, even at high network confidence.
  {
    Tracker tracker(fixture.path.string(), solver);
    std::uint64_t id = 0;
    int blocked_size_votes = 0;
    for (int frame = 0; frame < 70; ++frame) {
      std::list<Armor> armors{observation(solver, a, frame < 10 || frame >= 60 ? 11 : 8)};
      const auto targets = tracker.track(armors, timestamp(frame));
      if (!frame) id = tracker.track_id();
      assert(targets.size() == 1 && tracker.track_id() == id);
      assert(targets.front().name == four && targets.front().armor_type == small);
      if (frame) assert(!tracker.diagnostics().rebuilt);
      blocked_size_votes += tracker.diagnostics().size_correction_rejected;
    }
    assert(blocked_size_votes >= 40 && tracker.identity_confirmed());
  }

  // From startup, three-frame class flicker used to cause a reset every three
  // frames. The first four-class detection anchors geometry and physical size.
  {
    Tracker tracker(fixture.path.string(), solver);
    std::uint64_t id = 0;
    for (int frame = 0; frame < 60; ++frame) {
      std::list<Armor> armors{observation(solver, a, (frame / 3) % 2 ? 8 : 11)};
      const auto targets = tracker.track(armors, timestamp(frame));
      assert(targets.size() == 1 && tracker.measurement_updated());
      if (frame == 0) id = tracker.track_id();
      assert(tracker.track_id() == id);
      if (frame >= 4) assert(tracker.state() == "tracking");
    }
    // Geometry can track while no class has earned enough consecutive evidence.
    assert(!tracker.identity_confirmed());
  }

  // A sustained same-size class correction changes the semantic attribute and
  // rebuilds the new vehicle model, while preserving the geometric track ID.
  {
    Tracker tracker(fixture.path.string(), solver);
    std::uint64_t id = 0;
    for (int frame = 0; frame < 20; ++frame) {
      std::list<Armor> armors{observation(solver, a, frame < 10 ? 11 : 10)};
      const auto targets = tracker.track(armors, timestamp(frame));
      assert(targets.size() == 1 && tracker.measurement_updated());
      if (!frame) id = tracker.track_id();
      assert(tracker.track_id() == id);
      if (frame < 14) assert(targets.front().name == four);
      else assert(targets.front().name == three && armors.front().name == three);
    }
    assert(tracker.identity_confirmed() && tracker.state() == "tracking");
  }

  // Another same-class physical object must not steal a live geometric ID,
  // even when detections reorder or the selected object briefly disappears.
  {
    Tracker tracker(fixture.path.string(), solver);
    std::uint64_t id = 0;
    for (int frame = 0; frame < 40; ++frame) {
      std::list<Armor> armors;
      const bool hide_a = frame >= 20 && frame < 24;
      if (frame % 2) armors.push_back(observation(solver, b));
      if (!hide_a) armors.push_back(observation(solver, a));
      if (!(frame % 2)) armors.push_back(observation(solver, b));
      const auto targets = tracker.track(armors, timestamp(frame));
      assert(targets.size() == 1);
      if (!frame) id = tracker.track_id();
      assert(tracker.track_id() == id);
      assert((tracker.primary_observation() != nullptr) == tracker.measurement_updated());
      if (hide_a) assert(!tracker.measurement_updated() && tracker.state() == "temp_lost");
      else assert(tracker.measurement_updated());
      const auto faces = targets.front().armor_xyza_list();
      const auto & face = faces[targets.front().last_id];
      assert((face.head<3>() - a).norm() < .03);
    }
    // A different class outside the continuity envelope is not rewritten.
    for (int frame = 40; frame < 44; ++frame) {
      std::list<Armor> armors{observation(solver, b, 10)};
      tracker.track(armors, timestamp(frame));
      assert(!tracker.measurement_updated() && armors.front().name == three);
    }
  }

  // Short coasting retains identity. A long loss retires it, so later physical
  // objects receive a new ID. Duplicate/old timestamps never update a filter.
  {
    Tracker tracker(fixture.path.string(), solver);
    std::uint64_t id = 0;
    for (int frame = 0; frame < 10; ++frame) {
      std::list<Armor> armors{observation(solver, a)};
      tracker.track(armors, timestamp(frame));
      id = tracker.track_id();
    }
    auto duplicate = std::list<Armor>{observation(solver, b)};
    tracker.track(duplicate, timestamp(9));
    assert(!tracker.measurement_updated() && tracker.last_associations().empty());
    assert(tracker.track_id() == id);
    for (int frame = 10; frame < 13; ++frame) {
      std::list<Armor> none;
      assert(tracker.track(none, timestamp(frame)).size() == 1);
      assert(!tracker.measurement_updated() && tracker.track_id() == id);
    }
    auto armors = std::list<Armor>{observation(solver, a)};
    tracker.track(armors, timestamp(13));
    assert(tracker.measurement_updated() && tracker.track_id() == id);
    for (int frame = 14; frame < 30; ++frame) {
      std::list<Armor> none;
      tracker.track(none, timestamp(frame));
    }
    assert(tracker.state() == "lost" && tracker.track_id() == 0);
    armors = {observation(solver, b)};
    tracker.track(armors, timestamp(30));
    assert(tracker.track_id() != 0 && tracker.track_id() != id);
  }

  // Fixed-seed independent corner noise tests observability, not just ideal
  // reprojection. A stationary one-face view must not invent a radius reset.
  for (double distance : {2.0, 3.0, 5.0, 7.0}) for (double sigma : {0.3, 1.0}) {
    Tracker tracker(fixture.path.string(), solver);
    std::mt19937 random(20261017);
    std::uint64_t id = 0;
    int misses = 0;
    for (int frame = 0; frame < 610; ++frame) {
      const Eigen::Vector3d position(distance, 0.0, 0.0);
      auto armors = std::list<Armor>{observation(
        solver, position, 11, frame < 10 ? nullptr : &random, sigma)};
      const auto targets = tracker.track(armors, timestamp(frame));
      assert(targets.size() == 1);
      if (!frame) id = tracker.track_id();
      if (tracker.track_id() != id) {
        std::cerr << "stationary noise reset: range=" << distance << " sigma=" << sigma << " frame=" << frame << '\n';
      }
      assert(tracker.track_id() == id);
      misses += !tracker.measurement_updated();
      assert(targets.front().ekf_x().allFinite() && !targets.front().diverged());
    }
    std::cout << "static noise range=" << distance << "m sigma=" << sigma
              << "px, noisy_frames=600, missed_measurements=" << misses << '\n';
  }

  // An independent phase from a different periodic face branch cannot borrow
  // the vehicle center/radii and gain prediction trust. Deliberately perturb
  // the actual EKF mean to exercise the cross-estimator disagreement boundary.
  {
    auto armor = observation(solver, a);
    solver.solve(armor);
    Target target(armor, timestamp(0), .2, 4, Eigen::VectorXd::Ones(11));
    for (int frame = 1; frame <= 40; ++frame) {
      target.predict(timestamp(frame));
      target.update(armor, 0);
    }
    assert(target.yaw_eskf_trusted() && target.federated_phase_consistent());
    auto & vehicle = const_cast<tools::ExtendedKalmanFilter &>(target.ekf());
    vehicle.x[6] += M_PI / 2.0;
    assert(!target.federated_phase_consistent() && !target.yaw_eskf_trusted());
    assert((target.ekf_x() - vehicle.x).norm() < 1e-12);
    const auto faces = target.association_armor_xyza_list();
    assert(std::abs(faces[0][3] - vehicle.x[6]) < 1e-12);
    assert(std::abs(faces[0][0] - (vehicle.x[0] - vehicle.x[8] * std::cos(vehicle.x[6]))) < 1e-12);
  }

  // Actual Solver -> Tracker rotation starts with an unknown radius (.20 m
  // prior, .30 m truth). One physical revolution is an observability warmup;
  // report any initial loss explicitly, then require uninterrupted geometric
  // identity and correct radius/rate during the remaining steady sequence.
  for (double rate : {-4.0, 1.8, 4.0}) {
    Tracker tracker(fixture.path.string(), solver);
    Tracker sampled_diagnostics(fixture.path.string(), solver);
    std::uint64_t id = 0, steady_id = 0;
    const int warmup_frames = static_cast<int>(std::ceil(2.0 * M_PI / std::abs(rate) / .01));
    int warmup_lost = 0, warmup_resets = 0, adaptive_frames = 0, whole_frames = 0;
    double radius = 0.0, omega = 0.0;
    for (int frame = 0; frame < 1200; ++frame) {
      std::list<Armor> armors;
      for (int face = 0; face < 4; ++face) {
        const double angle = std::remainder(rate * frame * .01 + face * M_PI / 2.0, 2.0 * M_PI);
        if (std::abs(angle) > 55.0 * M_PI / 180.0) continue;
        const Eigen::Vector3d position(3.3 - .3 * std::cos(angle), -.3 * std::sin(angle), 0.0);
        armors.push_back(observation(solver, position, 11, nullptr, 0.0, angle));
      }
      auto sampled_armors = armors;
      sampled_diagnostics.set_association_diagnostics_enabled(frame % 3 == 0);
      const auto time = start + std::chrono::milliseconds(10 * frame);
      const auto targets = tracker.track(armors, time);
      const auto sampled_targets = sampled_diagnostics.track(sampled_armors, time);
      assert(targets.size() == sampled_targets.size());
      assert(tracker.state() == sampled_diagnostics.state());
      assert(tracker.track_id() == sampled_diagnostics.track_id());
      assert(tracker.primary_face_id() == sampled_diagnostics.primary_face_id());
      assert(tracker.measurement_updated() == sampled_diagnostics.measurement_updated());
      if (!targets.empty()) {
        assert((targets.front().ekf().x - sampled_targets.front().ekf().x).norm() < 1e-12);
        assert((targets.front().ekf().P - sampled_targets.front().ekf().P).norm() < 1e-12);
      }
      if (targets.empty()) {
        assert(frame < warmup_frames);
        ++warmup_lost;
        std::cout << "rotation initialization loss rate=" << rate << " frame=" << frame << '\n';
        continue;
      }
      if (id && tracker.track_id() != id) {
        assert(frame < warmup_frames);
        ++warmup_resets;
      }
      id = tracker.track_id();
      if (frame >= warmup_frames) {
        if (!steady_id) steady_id = id;
        assert(id == steady_id);
      }
      adaptive_frames += targets.front().center_acceleration_variance() == 10.0;
      if (targets.front().whole_rotation_trusted()) {
        if (whole_frames++ == 0) {
          auto invalid = targets.front(); invalid.jumped = false;
          assert(!invalid.whole_rotation_trusted());
          invalid = targets.front();
          const_cast<tools::ExtendedKalmanFilter &>(invalid.ekf()).P(7,7) = 100.;
          assert(!invalid.whole_rotation_trusted());
          invalid = targets.front();
          const_cast<tools::ExtendedKalmanFilter &>(invalid.ekf()).P(6,6) = 1.;
          assert(!invalid.whole_rotation_trusted());
        }
      }
      radius = targets.front().ekf_x()[8];
      omega = targets.front().ekf_x()[7];
    }
    assert(adaptive_frames > 100 && whole_frames > 100);
    assert(std::abs(radius - .3) < .03 && std::abs(omega - rate) < .1);
    std::cout << "real tracker rotation rate=" << rate << " frames=1200 radius=" << radius
              << " omega=" << omega << " warmup_lost=" << warmup_lost
              << " warmup_resets=" << warmup_resets << " steady_lost=0 steady_resets=0\n";
  }
  std::cout << "tracker_pipeline_test: PASS; real geometry, class flicker, persistent identity, two objects, loss and noise\n";
}
