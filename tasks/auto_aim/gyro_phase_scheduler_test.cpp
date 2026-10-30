#include "gyro_phase_scheduler.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
void cross(auto_aim::GyroPhaseScheduler & scheduler, double t, double omega)
{
  // The folded phase wraps at +/-45 degrees between faces.  This large jump
  // must not be mistaken for the physical zero crossing.
  scheduler.observe(t - 0.012, -0.70, omega);
  if (omega > 0.0) {
    scheduler.observe(t - 0.004, -0.02, omega);
    scheduler.observe(t + 0.004, +0.02, omega);
  } else {
    scheduler.observe(t - 0.004, +0.02, omega);
    scheduler.observe(t + 0.004, -0.02, omega);
  }
}

void acquire(
  auto_aim::GyroPhaseScheduler & scheduler, double first_t,
  double period, double omega)
{
  for (int i = 0; i < 6; ++i) cross(scheduler, first_t + i * period, omega);
}
}  // namespace

int main()
{
  constexpr double omega = 5.0;
  const double period = (M_PI / 2.0) / omega;

  auto_aim::GyroPhaseScheduler scheduler;
  acquire(scheduler, 1.0, period, omega);
  assert(scheduler.period_valid());
  assert(scheduler.locked());
  assert(std::abs(scheduler.face_period_s() - period) < 1e-6);

  // A missed crossing is normalized to two face periods instead of doubling
  // the estimated period.
  cross(scheduler, 1.0 + 7.0 * period, omega);
  assert(std::abs(scheduler.face_period_s() - period) < 1e-6);

  const double now = 1.0 + 7.0 * period + 0.010;
  const auto safe = scheduler.solve(
    now, 0.154, 0.134, 0.174, 0.155, 0.005,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(safe.valid);
  assert(safe.interval_safe);
  assert(safe.time_until_command_s >= 0.0);
  assert(safe.time_until_command_s < period);

  const auto unsafe = scheduler.solve(
    now, 0.154, 0.034, 0.254, 0.155, 0.020,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(unsafe.valid);
  assert(!unsafe.interval_safe);

  const auto stale = scheduler.solve(
    now + 4.0 * period, 0.154, 0.134, 0.174, 0.155, 0.005,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(!stale.valid);

  const auto invalid_radius = scheduler.solve(
    now, 0.154, 0.134, 0.174, 0.155, 0.005,
    0.0, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(!invalid_radius.valid);

  // Reverse rotation uses the opposite physical zero-crossing edge.
  auto_aim::GyroPhaseScheduler reverse_scheduler;
  acquire(reverse_scheduler, 2.0, period, -omega);
  assert(reverse_scheduler.locked());
  assert(std::abs(reverse_scheduler.face_period_s() - period) < 1e-6);

  // Fixed 3m bench: schedule and send angle must describe the same impact.
  // Exercise both directions, different image ages and face-equivalent angles.
  constexpr double bench_speed = 19.8;
  constexpr double bench_delay = .085;
  constexpr double bench_flight = 3.0 / bench_speed;
  const double bench_rate = (M_PI / 2.0) / .362044784;
  for (double sign : {-1.0, 1.0}) {
    auto_aim::GyroPhaseScheduler bench;
    acquire(bench, 60.0, (M_PI / 2.0) / bench_rate, sign * bench_rate);
    const double bench_now = 60.0 + 5.0 * .362044784 + .02;
    const auto clock = bench.solve(bench_now, bench_delay, .065, .105,
      bench_flight, .005, .28, .135, 0, .7 * M_PI / 180.0);
    assert(clock.valid);
    for (double age : {.015, .040, .073}) {
      for (double phase : {-1.2, -.2, 0.0, .3, 1.4}) {
        const auto aligned = auto_aim::GyroPhaseScheduler::align_to_geometry(
          clock, bench_now, bench_now-age, phase, sign * bench_rate);
        assert(aligned.valid && aligned.geometry_aligned);
        assert(aligned.time_until_command_s >= 0.0 &&
          aligned.time_until_command_s <= aligned.face_period_s + 1e-7);
        assert(std::abs(aligned.total_lead_s-(bench_delay+bench_flight))<1e-9);
        assert(std::abs(aligned.command_phase_error_at(aligned.command_due_time_s))<1e-9);
        assert(std::abs(aligned.phase_at(aligned.command_due_time_s+aligned.total_lead_s))<1e-9);
        assert(std::abs(aligned.command_phase_error_at(aligned.command_due_time_s+.010)-
          sign * bench_rate * .010)<1e-9);
        const auto equivalent = auto_aim::GyroPhaseScheduler::align_to_geometry(
          clock, bench_now, bench_now-age, phase + M_PI / 2.0, sign * bench_rate);
        assert(std::abs(equivalent.command_due_time_s-aligned.command_due_time_s)<1e-9);
      }
    }
    const auto invalid = auto_aim::GyroPhaseScheduler::align_to_geometry(
      clock, bench_now, bench_now+.001, .2, sign * bench_rate);
    assert(!invalid.valid);
    const auto nan_phase = auto_aim::GyroPhaseScheduler::align_to_geometry(
      clock, bench_now, bench_now-.02, NAN, sign * bench_rate);
    assert(!nan_phase.valid);

    // The target builder supplies 4.87rad/s. Use it consistently for timing,
    // phase and uncertainty while retaining the independent observed PLL rate.
    constexpr double fixed_rate = 4.87;
    const double observed_period_before = bench.face_period_s();
    const auto fixed_clock = bench.solve(bench_now, bench_delay, .065, .105,
      bench_flight, .005, .28, .135, 0, .7 * M_PI / 180.0, fixed_rate);
    assert(fixed_clock.valid && std::abs(fixed_clock.omega_rad_s-fixed_rate)<1e-12);
    assert(std::abs(fixed_clock.face_period_s-(M_PI/2.0)/fixed_rate)<1e-12);
    assert(bench.face_period_s()==observed_period_before);
    assert(std::abs(fixed_clock.phase_uncertainty_rad/clock.phase_uncertainty_rad-
      fixed_rate/clock.omega_rad_s)<1e-10);
    for (double phase : {-1.2, -.2, 0.0, .3, 1.4}) {
      const auto fixed = auto_aim::GyroPhaseScheduler::align_to_geometry(
        fixed_clock, bench_now, bench_now-.04, phase, sign * bench_rate);
      assert(fixed.valid && fixed.geometry_aligned);
      assert(std::abs(fixed.signed_omega_rad_s-sign*fixed_rate)<1e-12);
      const double expected = std::remainder(-sign*fixed_rate*(bench_delay+bench_flight),M_PI/2.0);
      assert(std::abs(fixed.command_phase_rad-expected)<1e-10);
      assert(std::abs(std::abs(expected)*180/M_PI-24.004)<.01);
      assert(std::abs(fixed.command_phase_error_at(fixed.command_due_time_s))<1e-9);
      assert(std::abs(fixed.phase_at(fixed.command_due_time_s+fixed.total_lead_s))<1e-9);
      assert(fixed.time_until_command_s>=0.0 && fixed.time_until_command_s<=fixed.face_period_s+1e-7);
    }
    const auto bad_fixed_rate = bench.solve(bench_now, bench_delay, .065, .105,
      bench_flight, .005, .28, .135, 0, .7*M_PI/180.0, NAN);
    assert(!bad_fixed_rate.valid);
  }

  // Paired long/short raw crossings reproduce the field-log failure mode.
  // The PLL phase anchor may move only a few milliseconds instead of following
  // the full +/-25 ms measurement jitter.
  auto_aim::GyroPhaseScheduler jitter_scheduler;
  const std::vector<double> jitter_s{0.0, 0.025, -0.025, 0.022, -0.020, 0.018, -0.018, 0.0};
  for (std::size_t i = 0; i < jitter_s.size(); ++i) {
    cross(jitter_scheduler, 4.0 + i * period + jitter_s[i], omega);
  }
  assert(jitter_scheduler.locked());
  assert(std::abs(jitter_scheduler.face_period_s() - period) < 0.010);
  const double ideal_last_crossing = 4.0 + (jitter_s.size() - 1) * period;
  const double anchor_error_s =
    jitter_scheduler.phase_anchor_time_s() - ideal_last_crossing;
  std::cout << "paired-jitter anchor error: " << anchor_error_s * 1000.0 << " ms\n";
  assert(std::abs(anchor_error_s) < 0.020);

  // A phase estimate that bounces back across zero after one real crossing
  // must not create a second 20--40 ms physical event or discard PLL lock.
  auto_aim::GyroPhaseScheduler duplicate_scheduler;
  acquire(duplicate_scheduler, 6.0, period, omega);
  assert(duplicate_scheduler.locked());
  const auto updates_before_duplicate = duplicate_scheduler.lock_updates();
  cross(duplicate_scheduler, 6.0 + 5.0 * period + 0.030, omega);
  assert(duplicate_scheduler.locked());
  assert(duplicate_scheduler.lock_updates() == updates_before_duplicate);

  // A single early/late crossing temporarily vetoes solve(), but the next
  // consistent crossing restores the mature lock without four fresh updates.
  auto_aim::GyroPhaseScheduler single_outlier_scheduler;
  acquire(single_outlier_scheduler, 7.0, period, omega);
  const double outlier_t = 7.0 + 6.0 * period + 0.100;
  cross(single_outlier_scheduler, outlier_t, omega);
  assert(single_outlier_scheduler.locked());
  const auto suspect_solution = single_outlier_scheduler.solve(
    outlier_t + 0.010, 0.154, 0.134, 0.174, 0.155, 0.005,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(!suspect_solution.valid);
  cross(single_outlier_scheduler, 7.0 + 7.0 * period, omega);
  assert(single_outlier_scheduler.locked());
  const auto recovered_solution = single_outlier_scheduler.solve(
    7.0 + 7.0 * period + 0.010,
    0.154, 0.134, 0.174, 0.155, 0.005,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(recovered_solution.valid);

  // Field replay regression: the measured face period is about 327 ms while
  // EKF omega implies about 320 ms.  The event median must remove that small
  // persistent frequency bias instead of accumulating a 90 ms phase error and
  // permanently dropping into the native fallback path.
  auto_aim::GyroPhaseScheduler biased_omega_scheduler;
  constexpr double field_period = 0.327;
  constexpr double ekf_period = 0.320;
  const double biased_omega = (M_PI / 2.0) / ekf_period;
  for (int i = 0; i < 120; ++i) {
    cross(biased_omega_scheduler, 8.0 + i * field_period, biased_omega);
  }
  assert(biased_omega_scheduler.locked());
  assert(std::abs(biased_omega_scheduler.face_period_s() - field_period) < 0.004);
  const double biased_anchor_error_s =
    biased_omega_scheduler.phase_anchor_time_s() - (8.0 + 119.0 * field_period);
  std::cout << "biased-omega anchor error: " << biased_anchor_error_s * 1000.0 << " ms\n";
  assert(std::abs(biased_anchor_error_s) < 0.020);
  assert(biased_omega_scheduler.phase_uncertainty_s() < 0.010);

  const auto field_solution = biased_omega_scheduler.solve(
    8.0 + 119.0 * field_period + 0.010,
    0.154, 0.134, 0.174, 0.155, 0.005,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(field_solution.valid);
  assert(std::isfinite(field_solution.phase_uncertainty_rad));

  // A real fast-to-slow transition must not alias one ~0.85 s slow interval
  // into three ~0.28 s fast periods merely because the previous PLL was fast.
  auto_aim::GyroPhaseScheduler transition_scheduler;
  acquire(transition_scheduler, 30.0, period, omega);
  constexpr double slow_omega = 1.85;
  const double slow_period = (M_PI / 2.0) / slow_omega;
  auto transition_observation = [&]() {
    transition_scheduler.observe(32.0 - 0.012, -0.70, slow_omega);
    transition_scheduler.observe(32.0 - 0.004, -0.02, slow_omega);
    return transition_scheduler.observe(32.0 + 0.004, +0.02, slow_omega);
  }();
  assert(transition_observation.omega_alias_guarded);
  assert(transition_observation.periods_elapsed == 1);
  for (int i = 1; i < 8; ++i) {
    cross(transition_scheduler, 32.0 + i * slow_period, slow_omega);
  }
  assert(transition_scheduler.locked());
  assert(std::abs(transition_scheduler.face_period_s() - slow_period) < 0.04);

  // A genuine phase discontinuity must temporarily unlock, establish a fresh
  // anchor from several consistent crossings, and then reacquire without a
  // mode reset.
  auto_aim::GyroPhaseScheduler relock_scheduler;
  acquire(relock_scheduler, 50.0, period, omega);
  bool saw_reacquire = false;
  for (int i = 6; i < 16; ++i) {
    const auto observation = [&]() {
      relock_scheduler.observe(50.0 + i * period + 0.100 - 0.012, -0.70, omega);
      relock_scheduler.observe(50.0 + i * period + 0.100 - 0.004, -0.02, omega);
      return relock_scheduler.observe(
        50.0 + i * period + 0.100 + 0.004, +0.02, omega);
    }();
    saw_reacquire = saw_reacquire || observation.reacquired;
  }
  assert(saw_reacquire);
  assert(relock_scheduler.locked());

  // A long rejected/missing-crossing gap must start unlocked in both
  // directions, then recover only after fresh consistent observations.
  for (double sign : {-1.0, 1.0}) {
    auto_aim::GyroPhaseScheduler gap_scheduler;
    const double signed_omega = sign * omega;
    acquire(gap_scheduler, 70.0, period, signed_omega);
    const auto old_cycle = gap_scheduler.crossing_cycle();
    const double resumed_t = 70.0 + 20.0 * period;
    cross(gap_scheduler, resumed_t, signed_omega);
    assert(!gap_scheduler.locked());
    assert(gap_scheduler.lock_updates() == 0);
    assert(gap_scheduler.crossing_cycle() > old_cycle);
    assert(!gap_scheduler.solve(
      resumed_t + .010, .035, .015, .055, .152, .005,
      .28, .135, .005, .01, 4.56).valid);
    cross(gap_scheduler, resumed_t + .030, signed_omega);
    assert(gap_scheduler.lock_updates() == 0);
    for (int i = 1; i <= 4; ++i) {
      cross(gap_scheduler, resumed_t + i * period, signed_omega);
      if (i < 4) assert(!gap_scheduler.locked());
    }
    assert(gap_scheduler.locked());
    assert(gap_scheduler.solve(
      resumed_t + 4.0 * period + .010, .035, .015, .055, .152, .005,
      .28, .135, .005, .01, 4.56).valid);
  }

  // Direct phase-window bench: both directions, image-age compensation,
  // 90-degree wrapping, write-only consumption, Schmitt rearm and stale veto.
  for (double sign : {-1.0, 1.0}) {
    auto_aim::PhaseWindowTrigger window;
    const double w = sign * 4.56;
    const double lead = .035 + 3.0/19.8;
    const double tol = 4.0 * M_PI/180.0;
    const double target = std::remainder(-w*lead, M_PI/2.0);
    const auto sample = [&](double now, double error_deg, double age = .03) {
      return window.observe(now, now-age,
        std::remainder(target + sign*error_deg*M_PI/180.0 - w*age, M_PI/2.0),
        w, lead, tol, .02, .1);
    };
    auto first = sample(100., -3.9);
    assert(first.valid);
    assert(std::abs(first.phase_at(100.+lead)) < tol);
    assert(sample(100.001, -3.5).hit_cycle == first.hit_cycle);
    // A throttled request did not consume this passage.
    assert(sample(100.002, -3.).valid);
    window.mark_fired();
    assert(!sample(100.003, +3.9).valid);
    assert(!sample(100.004, +6.).valid);
    assert(!sample(100.005, +2.).valid); // Jitter cannot rearm within +/-8 deg.
    assert(!sample(100.006, +10.).valid);
    const auto next = sample(100.35, -2.);
    assert(next.valid && next.hit_cycle > first.hit_cycle);
    assert(!sample(100.36, 0., .11).valid);
    const auto before_center = sample(100.4, -3.);
    assert(before_center.valid);
    const double e = sign * before_center.command_phase_error_at(100.4);
    const double remaining = (tol-e)/std::abs(w);
    assert(remaining > .02); // Before-center entry must retain the full forward window.
    assert(!window.observe(100.5, 100.51, 0., w, lead, tol, .02, .1).valid);
  }

  // One-sided policy: preparation before zero never grants permission to
  // transmit before zero. Both signs use progress along the actual rotation.
  for (double sign : {-1.0, 1.0}) {
    auto_aim::PhaseWindowTrigger window;
    const double w = sign * 4.56;
    const double lead = .035 + 3.0/19.8;
    const double tol = 4.0 * M_PI/180.0;
    const double target = std::remainder(-w*lead, M_PI/2.0);
    const auto sample = [&](double now, double progress_deg) {
      return window.observe(now, now-.03,
        std::remainder(target + sign*progress_deg*M_PI/180.0 - w*.03, M_PI/2.0),
        w, lead, tol, .02, .1, true);
    };
    const auto early = sample(200., -3.);
    assert(early.valid && early.command_due_time_s > 200.);
    assert(!early.command_phase_in_window_at(200., tol, true));
    assert(early.command_phase_in_window_at(200., tol, false));
    assert(!early.command_phase_in_window_at(early.command_due_time_s - .00001, tol, true));
    assert(early.command_phase_in_window_at(early.command_due_time_s + .00001, tol, true));
    const auto passed = sample(200.02, +2.);
    assert(passed.valid && passed.time_until_command_s == 0.0);
    assert(passed.command_phase_in_window_at(200.02, tol, true));
    const auto edge = sample(200.021, +3.9);
    assert(edge.valid && edge.command_phase_in_window_at(200.021, tol, true));
    assert(!sample(200.022, +4.1).valid);
    window.mark_fired();
    assert(!sample(200.023, +1.).valid);
    assert(!sample(200.024, +10.).valid);
    const auto next = sample(200.35, -2.);
    assert(next.valid && next.hit_cycle > early.hit_cycle);
    assert(!next.command_phase_in_window_at(200.35, tol, true));
  }


  // Exact reference speed needs no clock correction. Both directions produce
  // the same deadlines, irrespective of delay and flight tuning.
  for (double sign : {-1., 1.}) {
    auto_aim::FixedCadenceTrigger cadence;
    const double w = sign*4.56, period = (M_PI/2)/4.56, offset = .25692;
    auto observe = [&](double now, double lead = .7765, double age = .025, int stride = 1) {
      const double sample = now-age;
      return cadence.observe(now,sample,std::remainder(w*(sample-100.),M_PI/2),
        w,lead,.07,.02,.1,offset,stride,.05,.002);
    };
    auto seeded = observe(100.025);
    assert(seeded.geometry_aligned && !seeded.valid);
    assert(std::abs(seeded.command_due_time_s-(100.+offset)) < 1e-10);
    double last_due = -1.; int fires = 0;
    for (int i=1; i<600; ++i) {
      const double now = 100.025 + i*.01;
      const auto slot = observe(now, i%2 ? .7765 : 1.3);
      if (!slot.valid) continue;
      assert(slot.command_due_time_s >= now-.002);
      assert(slot.command_due_time_s <= now+.05);
      assert(slot.command_phase_in_window_at(slot.command_due_time_s+.00001,.07,true));
      if (last_due>0.) assert(std::abs(slot.command_due_time_s-last_due-period)<1e-9);
      last_due=slot.command_due_time_s; ++fires; cadence.mark_fired();
      assert(!observe(now).valid); // no repeat write for the same deadline
    }
    assert(fires>=17);
    assert(!observe(107.,.7765,.11).valid); // stale image clears the anchor
    assert(!observe(107.1,.7765,-.01).valid); // future image
    assert(!cadence.observe(108.,108.,0.,w,.77,.07,.02,.1,NAN,1,.05,.002).valid);
  }
  // Skipped slots are never caught up; a lost observation requires a fresh
  // centre passage. Reset/reacquisition cycle ids never reuse an old shot id.
  {
    auto_aim::FixedCadenceTrigger cadence;
    auto slot = cadence.observe(10.,10.,0.,4.56,.77,.07,.02,.1,.1,1,.05,.002);
    const auto old_cycle=slot.hit_cycle;
    slot=cadence.observe(10.3,10.3,4.56*.3,4.56,.77,.07,.02,.1,.1,1,.05,.002);
    assert(!slot.valid && !slot.geometry_aligned); // over-age stream gap, far from centre
    slot=cadence.observe(11.,11.,0.,4.56,.77,.07,.02,.1,.02,2,.05,.002);
    assert(slot.valid && slot.hit_cycle>old_cycle);
    const double due=slot.command_due_time_s;
    cadence.mark_fired();
    slot=cadence.observe(11.01,11.01,4.56*.01,4.56,.77,.07,.02,.1,.02,2,.05,.002);
    assert(std::abs(slot.command_due_time_s-due-2*(M_PI/2)/4.56)<1e-10);
    // A fresh stream can still miss a slot: discard it, never send it late.
    auto_aim::FixedCadenceTrigger late;
    late.observe(20.,20.,0.,4.56,.77,.07,.02,.1,.02,1,.05,.002);
    slot=late.observe(20.023,20.023,4.56*.023,4.56,.77,.07,.02,.1,.02,1,.05,.002);
    assert(!slot.valid && slot.command_due_time_s>20.3);
    assert(!late.observe(20.024,20.024,0.,-4.56,.77,.07,.02,.1,.02,0,.05,.002).valid);
  }

  // Rotating-face hand-over must not clear the clock. No observe call occurs
  // while measurements are rejected; new fresh samples retain the clock.
  // Noisy visual phase is not a duplicate TX-window gate; mature crossings may
  // make bounded clock corrections rather than clearing/restarting the cadence.
  for (double sign : {-1.,1.}) {
    auto_aim::FixedCadenceTrigger cadence;
    const double w=sign*4.56, period=(M_PI/2)/4.56, offset=.25692;
    cadence.observe(200.025,200.,0.,w,.77,.07,.02,.1,offset,1,.05,.002,.25);
    double last_due=-1.;int count=0;
    for(int i=1;i<=500;++i) {
      if(i%9==0 || i%9==1) continue; // transient rejected observations
      const double now=200.025+i*.02, sample=now-.025;
      const double jitter=.12*std::sin(i*.4); // up to 6.9 deg > old 4 deg veto
      auto slot=cadence.observe(now,sample,w*(sample-200.)+jitter,w,
        .77,.07,.02,.1,offset,1,.05,.002,.25);
      assert(slot.geometry_aligned);
      assert(cadence.command_interval_s() >= .9*period);
      assert(cadence.command_interval_s() <= 1.1*period);
      if(!slot.valid) continue;
      assert(slot.command_phase_in_window_at(slot.command_due_time_s+.00001,.07,true));
      if(last_due>0.) assert(slot.command_due_time_s-last_due >= .85*period);
      last_due=slot.command_due_time_s;cadence.mark_fired();++count;
    }
    assert(count>=25); // near the 29 nominal slots despite periodic hand-over
    assert(std::isfinite(cadence.visual_phase_error_rad()));
    // Extended observation loss cannot keep the old grid armed.
    auto lost=cadence.observe(212.,212.,.5,w,.77,.07,.02,.1,offset,1,.05,.002,.25);
    assert(!lost.valid && !lost.geometry_aligned);
  }

  // The field failure: the physical rate is constant at 4.50 rad/s while the
  // configured seed is 4.56.  At both signs and strides, robust image crossings
  // must remove the accumulating phase error without changing the tuned 410 ms
  // offset, repeatedly reseeding, or producing duplicate/catch-up shots.
  for (double sign : {-1., 1.}) for (int stride : {1, 2}) {
    auto_aim::FixedCadenceTrigger cadence;
    const double nominal_w=sign*4.56, real_w=sign*4.50;
    const double real_face_period=(M_PI/2.)/4.50, offset=.410;
    double last_due=-1., previous_interval=0., max_settled_error=0.;
    int fires=0, settled_fires=0;
    std::int64_t last_cycle=-1;
    for(int i=0;i<=8000;++i) {
      if(i%31==12 || i%31==13) continue; // short hand-over gaps
      const double elapsed=i*.01, sample=300.+elapsed, now=sample+.025;
      double phase=real_w*elapsed+.004*std::sin(i*.37);
      if(i>0 && i%503==0) phase+=.6; // isolated impossible-looking phase sample
      const auto slot=cadence.observe(now,sample,phase,nominal_w,
        .77,.07,.02,.1,offset,stride,.05,.002,.25);
      const double interval=cadence.command_interval_s();
      assert(slot.geometry_aligned);
      if(previous_interval>0.)
        assert(std::abs(interval-previous_interval)<=.005*previous_interval+1e-10);
      previous_interval=interval;
      assert(std::abs(cadence.last_phase_adjustment_s())<=.003+1e-12);
      if(!slot.valid) continue;
      assert(slot.hit_cycle>last_cycle);
      if(last_due>0.) assert(slot.command_due_time_s-last_due>.85*stride*real_face_period);
      if(elapsed>25.) {
        const double error=std::abs(std::remainder(
          real_w*(slot.command_due_time_s-300.-offset),M_PI/2.));
        max_settled_error=std::max(max_settled_error,error);
        ++settled_fires;
      }
      last_due=slot.command_due_time_s;last_cycle=slot.hit_cycle;
      cadence.mark_fired();++fires;
      assert(!cadence.observe(now,sample,phase,nominal_w,
        .77,.07,.02,.1,offset,stride,.05,.002,.25).valid);
    }
    std::cout << "calibrated cadence sign=" << sign << " stride=" << stride
      << " interval_ms=" << cadence.command_interval_s()*1000.
      << " max_settled_phase_deg=" << max_settled_error*180./M_PI
      << " updates=" << cadence.calibration_updates() << " shots=" << fires << '\n';
    assert(cadence.calibration_updates()>40);
    assert(std::abs(cadence.command_interval_s()/stride-real_face_period)<.0015);
    assert(max_settled_error<.025);
    assert(fires>80 && settled_fires>50);
    const auto lost=cadence.observe(382.,382.,.5,nominal_w,
      .77,.07,.02,.1,offset,stride,.05,.002,.25);
    assert(!lost.valid && !lost.geometry_aligned && cadence.calibration_updates()==0);
  }

  // Field-like alternating early/late center estimates must not turn their
  // adjacent long/short intervals into an oscillating calibrated frequency.
  {
    auto_aim::FixedCadenceTrigger cadence;
    const double real_face_period=(M_PI/2.)/4.50;
    double max_settled_error=0.;int settled_fires=0;
    for(int i=0;i<8000;++i) {
      const double elapsed=i*.01,sample=400.+elapsed;
      const double phase=-4.50*elapsed+.10*std::cos(M_PI*elapsed/real_face_period);
      const auto slot=cadence.observe(sample+.025,sample,phase,-4.56,
        .77,.07,.02,.1,.410,2,.05,.002,.25);
      if(!slot.valid)continue;
      if(elapsed>25.) {
        max_settled_error=std::max(max_settled_error,std::abs(std::remainder(
          -4.50*(slot.command_due_time_s-400.-.410),M_PI/2.)));
        ++settled_fires;
      }
      cadence.mark_fired();
    }
    assert(settled_fires>60 && max_settled_error<.035);
    assert(std::abs(cadence.command_interval_s()/2.-real_face_period)<.0005);
  }

  // Several minutes of exact observations must not amplify a tiny frequency
  // update by multiplying it by the entire elapsed time from the first anchor.
  {
    auto_aim::FixedCadenceTrigger cadence;
    double last_due=-1., max_gap=0.; int fires=0;
    for(int i=0;i<18000;++i) {
      const double elapsed=i*.02, sample=500.+elapsed, now=sample+.025;
      const auto slot=cadence.observe(now,sample,-4.50*elapsed,-4.56,
        .77,.07,.02,.1,.410,2,.05,.002,.25);
      if(!slot.valid) continue;
      if(last_due>0.) max_gap=std::max(max_gap,slot.command_due_time_s-last_due);
      last_due=slot.command_due_time_s;cadence.mark_fired();++fires;
    }
    assert(fires>500 && max_gap<.75);
  }

  // A coherent crossing sequence can move by >22.5 degrees after acquisition.
  // Rejecting its phase residual must not also freeze frequency/phase learning
  // forever. This reproduces the long flat calibration counter in field logs.
  for (double sign : {-1., 1.}) {
    auto_aim::FixedCadenceTrigger cadence;
    const double w=sign*4.5, seed=sign*4.56, shift=sign*30.*M_PI/180.;
    double worst_after_recovery=0.; int recovered=0;
    for(int i=0;i<8000;++i) {
      const double elapsed=i*.01, sample=900.+elapsed;
      const double phase=w*elapsed+(elapsed>=15.?shift:0.);
      const auto slot=cadence.observe(sample+.025,sample,phase,seed,
        .77,.07,.02,.1,.460,2,.05,.002,.25);
      if(!slot.valid) continue;
      if(elapsed>45.) {
        const double error=std::abs(std::remainder(
          w*(slot.command_due_time_s-900.-.460)+shift,M_PI/2.));
        worst_after_recovery=std::max(worst_after_recovery,error); ++recovered;
      }
      cadence.mark_fired();
    }
    std::cout << "persistent phase shift sign=" << sign
      << " recovered_error_deg=" << worst_after_recovery*180./M_PI
      << " updates=" << cadence.calibration_updates() << std::endl;
    assert(recovered>30 && worst_after_recovery<.035);
    assert(cadence.large_residual_updates()>0);
  }

  std::cout << "gyro_phase_scheduler_test: PASS\n";
  return 0;
}
