//
// Copyright 2026 Pavan Patil
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @Author: Pavan Patil
// @Date: 2026-10-23
// @Description: Unit tests C5 to C7 and C9 for the docking state machine, driven by a fake clock.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "dockbot_docking/state_machine.hpp"
#include "kinematic_sim.hpp"

using dockbot_docking::DockingStateMachine;
using dockbot_docking::GoalRequest;
using dockbot_docking::MachineParams;
using dockbot_docking::MarkerObservation;
using dockbot_docking::Outcome;
using dockbot_docking::State;
using dockbot_docking::StepInput;
using dockbot_docking::StepOutput;
using dockbot_docking::testing::CameraModel;
using dockbot_docking::testing::KinematicSim;
using dockbot_docking::testing::Pose2D;

namespace
{

constexpr int kGoal = 2;
constexpr double kDt = 0.05;

/// Marker pose in base_link at distance d along the normal and lateral error e_y, facing square.
Eigen::Isometry3d pose_at(double d, double e_y)
{
  KinematicSim sim(Pose2D{}, Eigen::Vector2d(d, e_y), M_PI, CameraModel{}, 1);
  return sim.true_pose();
}

/// Drives a state machine with a fake clock and scripted detections.
struct Harness
{
  explicit Harness(MachineParams p = MachineParams())
  : machine((p.pose_filter_alpha = 1.0, p)) {}

  /// One control step at the next tick; the frame holds the given markers (or none).
  StepOutput tick(const std::vector<MarkerObservation> & markers, bool new_frame = true)
  {
    now += kDt;
    StepInput in;
    in.new_frame = new_frame;
    in.markers = markers;
    last = machine.step(in, now);
    return last;
  }

  /// One step with the goal marker seen at (d, e_y).
  StepOutput see(double d, double e_y, int id = kGoal)
  {
    return tick({MarkerObservation{id, pose_at(d, e_y), now + kDt}});
  }

  /// One step with a frame that contains no markers.
  StepOutput blank() {return tick({});}

  /// Feed the goal marker for n steps and return the last output.
  StepOutput see_n(int n, double d, double e_y)
  {
    StepOutput o;
    for (int i = 0; i < n; ++i) {
      o = see(d, e_y);
    }
    return o;
  }

  DockingStateMachine machine;
  double now = 0.0;
  StepOutput last;
};

GoalRequest goal(double dock_distance = 0.0, double timeout = 0.0)
{
  GoalRequest g;
  g.marker_id = kGoal;
  g.dock_distance = dock_distance;
  g.timeout = timeout;
  return g;
}

}  // namespace

// C5: idle until a goal arrives, and one goal at a time.
TEST(StateMachine, IdleUntilGoalAndRejectsSecondGoal)
{
  Harness h;
  EXPECT_EQ(h.blank().state, State::IDLE);
  EXPECT_TRUE(h.machine.start_goal(goal(), h.now));
  EXPECT_EQ(h.blank().state, State::SEARCH);
  EXPECT_FALSE(h.machine.start_goal(goal(), h.now));
}

// C5: SEARCH leaves only after acquire_frames consecutive detections.
TEST(StateMachine, AcquireNeedsConsecutiveFrames)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  EXPECT_EQ(h.see(2.0, 0.0).state, State::SEARCH);
  EXPECT_EQ(h.see(2.0, 0.0).state, State::SEARCH);
  EXPECT_EQ(h.blank().state, State::SEARCH);  // a miss resets the count
  EXPECT_EQ(h.see(2.0, 0.0).state, State::SEARCH);
  EXPECT_EQ(h.see(2.0, 0.0).state, State::SEARCH);
  const StepOutput o = h.see(2.0, 0.0);
  EXPECT_EQ(o.state, State::APPROACH);
  EXPECT_GT(o.cmd.v, 0.0);
}

// C5: the search ends in FAILED with TARGET_NOT_FOUND when nothing is ever seen.
TEST(StateMachine, SearchTimeoutFailsNotFound)
{
  MachineParams p;
  p.search_timeout = 5.0;
  Harness h(p);
  h.machine.start_goal(goal(), h.now);
  StepOutput o;
  while (h.now < 5.0 - 2 * kDt) {
    o = h.blank();
    ASSERT_EQ(o.state, State::SEARCH);
  }
  while (!o.finished && h.now < 10.0) {
    o = h.blank();
  }
  EXPECT_EQ(o.state, State::FAILED);
  EXPECT_EQ(h.machine.result().outcome, Outcome::TARGET_NOT_FOUND);
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(o.cmd.w, 0.0);
  EXPECT_EQ(h.blank().state, State::IDLE);
}

// C5: APPROACH to FINAL when inside final_range with the gates passing.
TEST(StateMachine, ApproachToFinalWhenGatesPass)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  EXPECT_EQ(h.see_n(3, 0.45, 0.0).state, State::FINAL);
}

// C5: APPROACH to RECOVER when inside final_range with the gates failing.
TEST(StateMachine, ApproachToRecoverWhenGatesFail)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  const StepOutput o = h.see_n(3, 0.45, 0.1);
  EXPECT_EQ(o.state, State::RECOVER);
  EXPECT_EQ(o.retries, 1);
  EXPECT_LE(o.cmd.v, 0.0);
}

// C5: still APPROACH while outside final_range, even with large errors.
TEST(StateMachine, StaysInApproachBeyondFinalRange)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  EXPECT_EQ(h.see_n(10, 1.5, 0.4).state, State::APPROACH);
}

// C5: FINAL to DOCKED exactly at the dock distance, then the result is reported once.
TEST(StateMachine, FinalToDocked)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(3, 0.45, 0.0).state, State::FINAL);
  EXPECT_EQ(h.see(0.2501, 0.0).state, State::FINAL);
  const StepOutput o = h.see(0.2499, 0.0);
  EXPECT_EQ(o.state, State::DOCKED);
  EXPECT_TRUE(o.finished);
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_EQ(h.machine.result().outcome, Outcome::DOCKED);
  EXPECT_NEAR(h.machine.result().final_distance, 0.0999, 1e-3);
  EXPECT_EQ(h.see(0.2499, 0.0).state, State::IDLE);
}

// C5: FINAL to RECOVER when the alignment degrades well past the gates.
TEST(StateMachine, FinalToRecoverOnLargeError)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(3, 0.45, 0.0).state, State::FINAL);
  EXPECT_EQ(h.see(0.45, 0.0).state, State::FINAL);
  // alpha 0.5 smoothing: feed a few frames so the filtered error passes twice the gate.
  StepOutput o;
  for (int i = 0; i < 6; ++i) {
    o = h.see(0.45, 0.1);
  }
  EXPECT_EQ(o.state, State::RECOVER);
}

// C5: RECOVER back to APPROACH once backed off to recover_range.
TEST(StateMachine, RecoverToApproach)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(3, 0.45, 0.1).state, State::RECOVER);
  EXPECT_EQ(h.see(0.9, 0.1).state, State::RECOVER);
  StepOutput o;
  for (int i = 0; i < 6; ++i) {
    o = h.see(2.1, 0.1);
  }
  EXPECT_EQ(o.state, State::APPROACH);
}

// C5: retries are counted, and the goal fails once they run out.
TEST(StateMachine, RetriesExceeded)
{
  MachineParams p;
  p.max_retries = 2;
  Harness h(p);
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(3, 0.45, 0.1).state, State::RECOVER);
  StepOutput o;
  for (int attempt = 2; attempt <= 3; ++attempt) {
    for (int i = 0; i < 6; ++i) {
      o = h.see(2.1, 0.1);
    }
    ASSERT_EQ(o.state, State::APPROACH);
    for (int i = 0; i < 6; ++i) {
      o = h.see(0.45, 0.1);
      if (o.state == State::RECOVER || o.finished) {
        break;
      }
    }
    if (attempt == 2) {
      ASSERT_EQ(o.state, State::RECOVER);
      EXPECT_EQ(o.retries, 2);
    }
  }
  EXPECT_EQ(o.state, State::FAILED);
  EXPECT_EQ(h.machine.result().outcome, Outcome::RETRIES_EXCEEDED);
}

// C5: a lost target is held briefly, then stopped, then searched for.
TEST(StateMachine, LostTargetTiming)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  StepOutput o = h.see_n(40, 2.5, 0.0);
  ASSERT_EQ(o.state, State::APPROACH);
  const double v_before = o.cmd.v;
  ASSERT_GT(v_before, 0.2);
  const double t_last = h.now;
  bool saw_stop = false;
  bool saw_search = false;
  double stop_time = -1.0;
  while (h.now - t_last < 1.5) {
    o = h.blank();
    const double lost = h.now - t_last;
    if (lost < 0.25) {
      EXPECT_EQ(o.state, State::APPROACH) << "lost " << lost;
      EXPECT_GT(o.cmd.v, 0.0) << "lost " << lost;
    }
    if (lost > 0.9 && lost < 0.99) {
      EXPECT_EQ(o.state, State::APPROACH) << "lost " << lost;
    }
    if (o.cmd.v == 0.0 && o.cmd.w == 0.0 && stop_time < 0.0) {
      stop_time = lost;
      saw_stop = true;
    }
    if (o.state == State::SEARCH) {
      saw_search = true;
      EXPECT_GT(h.now - t_last, 1.0 - 1e-9);
      break;
    }
  }
  EXPECT_TRUE(saw_stop);
  EXPECT_TRUE(saw_search);
}

// C5: cancel ends the goal with CANCELED and a zero command.
TEST(StateMachine, CancelEndsGoal)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(10, 2.0, 0.0).state, State::APPROACH);
  h.machine.cancel();
  const StepOutput o = h.see(2.0, 0.0);
  EXPECT_EQ(o.state, State::FAILED);
  EXPECT_TRUE(o.finished);
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_EQ(h.machine.result().outcome, Outcome::CANCELED);
}

// C5: the overall timeout ends the goal from any active state.
TEST(StateMachine, OverallTimeout)
{
  Harness h;
  h.machine.start_goal(goal(0.0, 3.0), h.now);
  StepOutput o;
  while (h.now < 4.0 && !o.finished) {
    o = h.see(2.0, 0.0);
  }
  EXPECT_TRUE(o.finished);
  EXPECT_EQ(h.machine.result().outcome, Outcome::TIMEOUT);
  EXPECT_NEAR(h.now, 3.0, 0.2);
}

// C6: detections that never contain the goal ID leave the machine searching.
TEST(StateMachine, OtherIdsAreIgnored)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  StepOutput o;
  for (int i = 0; i < 200; ++i) {
    o = h.tick({
      MarkerObservation{0, pose_at(0.5, 0.0), h.now + kDt},
      MarkerObservation{1, pose_at(1.0, 0.0), h.now + kDt},
      MarkerObservation{3, pose_at(2.0, 0.0), h.now + kDt}});
    ASSERT_EQ(o.state, State::SEARCH);
  }
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_NEAR(o.cmd.w, h.machine.params().control.search_angular_speed, 1e-12);
  EXPECT_FALSE(o.target_visible);
}

// C6: a wrong ID next to the goal does not disturb the approach.
TEST(StateMachine, GoalChosenAmongSeveral)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  StepOutput o;
  for (int i = 0; i < 3; ++i) {
    o = h.tick({
      MarkerObservation{1, pose_at(0.5, 0.3), h.now + kDt},
      MarkerObservation{kGoal, pose_at(2.0, 0.0), h.now + kDt}});
  }
  ASSERT_EQ(o.state, State::APPROACH);
  EXPECT_NEAR(o.errors.d, 2.0, 1e-9);
}

// C7: a stale pose counts as no detection, and the command goes to zero.
TEST(StateMachine, StalePoseIsNoDetection)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(3, 0.45, 0.0).state, State::FINAL);
  ASSERT_GT(h.see(0.45, 0.0).cmd.v, 0.0);
  StepOutput o;
  const double t0 = h.now;
  while (h.now - t0 < 0.4) {
    o = h.tick({MarkerObservation{kGoal, pose_at(0.5, 0.0), h.now - 1.0}});
  }
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(o.cmd.w, 0.0);
  EXPECT_FALSE(o.target_visible);
}

// C7: a NaN pose counts as no detection, and the command goes to zero.
TEST(StateMachine, NanPoseIsNoDetection)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(3, 0.45, 0.0).state, State::FINAL);
  Eigen::Isometry3d bad = pose_at(0.5, 0.0);
  bad.translation().x() = std::numeric_limits<double>::quiet_NaN();
  StepOutput o;
  const double t0 = h.now;
  while (h.now - t0 < 0.4) {
    o = h.tick({MarkerObservation{kGoal, bad, h.now}});
    ASSERT_TRUE(std::isfinite(o.cmd.v));
    ASSERT_TRUE(std::isfinite(o.cmd.w));
  }
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(o.cmd.w, 0.0);
}

// C9: an explicit dock distance is used; zero falls back to the parameter.
TEST(StateMachine, GoalDockDistance)
{
  {
    Harness h;
    h.machine.start_goal(goal(0.20), h.now);
    EXPECT_DOUBLE_EQ(h.machine.effective_dock_distance(), 0.20);
    ASSERT_EQ(h.see_n(3, 0.45, 0.0).state, State::FINAL);
    EXPECT_EQ(h.see(0.36, 0.0).state, State::FINAL);
    EXPECT_EQ(h.see(0.34, 0.0).state, State::DOCKED);
  }
  {
    Harness h;
    h.machine.start_goal(goal(0.0), h.now);
    EXPECT_DOUBLE_EQ(h.machine.effective_dock_distance(), 0.10);
    ASSERT_EQ(h.see_n(3, 0.45, 0.0).state, State::FINAL);
    EXPECT_EQ(h.see(0.34, 0.0).state, State::FINAL);
    EXPECT_EQ(h.see(0.24, 0.0).state, State::DOCKED);
  }
}

// A new goal is accepted on the cycle right after a result, without waiting for the idle step.
TEST(StateMachine, NewGoalRightAfterResult)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  h.machine.cancel();
  ASSERT_TRUE(h.blank().finished);
  EXPECT_FALSE(h.machine.active());
  EXPECT_TRUE(h.machine.start_goal(goal(), h.now));
  EXPECT_EQ(h.blank().state, State::SEARCH);
}

// After a re-acquire, a dropout holds the command just issued, not one from before the loss.
// The next step comes 0.2 s later (a slow cycle), so no fresh-pose step refreshes it first.
TEST(StateMachine, HoldAfterReacquireUsesNewCommand)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(40, 2.5, 0.0).state, State::APPROACH);  // issued 0.3 m/s before the loss
  while (h.blank().state != State::SEARCH) {
  }
  ASSERT_EQ(h.see_n(3, 0.6, 0.0).state, State::APPROACH);  // new command: about 0.1 m/s
  h.now += 0.2 - kDt;
  const StepOutput o = h.blank();
  EXPECT_EQ(o.state, State::APPROACH);
  EXPECT_LT(o.cmd.v, 0.09);  // holding the old 0.3 m/s would give about 0.125
}

// The final-approach decision uses range to the marker, so a robot still far off to the side keeps
// approaching and does not waste a retry.
TEST(StateMachine, FarOffToTheSideKeepsApproaching)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  const StepOutput o = h.see_n(10, 0.45, 1.0);
  EXPECT_EQ(o.state, State::APPROACH);
  EXPECT_EQ(o.retries, 0);
}

// Losing the marker in FINAL backs straight off first and only then turns in place.
TEST(StateMachine, LostInFinalBacksOffBeforeSearching)
{
  Harness h;
  h.machine.start_goal(goal(), h.now);
  ASSERT_EQ(h.see_n(3, 0.45, 0.0).state, State::FINAL);
  StepOutput o;
  double t_lost = h.now;
  while (h.now - t_lost < 1.3) {
    o = h.blank();
  }
  ASSERT_EQ(o.state, State::SEARCH);
  EXPECT_LT(o.cmd.v, 0.0);
  EXPECT_NEAR(o.cmd.w, 0.0, 1e-9);
  while (h.now - t_lost < 8.0) {
    o = h.blank();
  }
  EXPECT_EQ(o.state, State::SEARCH);
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_NEAR(o.cmd.w, h.machine.params().control.search_angular_speed, 1e-9);
}
