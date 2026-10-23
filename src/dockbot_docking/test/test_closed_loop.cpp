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
// @Description: Closed-loop tests CL1 to CL6: state machine driving a kinematic robot.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
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

constexpr double kDt = 0.05;
constexpr int kGoal = 2;

/// One recorded sample of a run, for the CSV output.
struct Sample
{
  double t, x, y, yaw;
  int state;
};

/// Everything a test needs to know about one closed-loop run.
struct RunResult
{
  bool finished = false;
  Outcome outcome = Outcome::CANCELED;
  bool docked_in_tolerance = false;
  bool crossed_plane = false;
  bool entered_recover = false;
  int retries = 0;
  double final_gap = 0.0;
  double final_lateral = 0.0;
  double final_heading = 0.0;
  double final_speed = 0.0;
  int approach_steps = 0;
  int approach_steps_in_view = 0;
  double travel_after_vanish = 0.0;
  double duration = 0.0;
  Pose2D start;
  std::vector<Sample> trajectory;
};

/// Run the full controller against the kinematic sim until the goal finishes or max_time passes.
RunResult run(
  const Pose2D & start, const CameraModel & camera, unsigned seed, double max_time = 150.0,
  MachineParams params = MachineParams(), bool record = false)
{
  // Marker at the world origin, normal along +x.
  KinematicSim sim(start, Eigen::Vector2d(0.0, 0.0), 0.0, camera, seed);
  DockingStateMachine machine(params);
  GoalRequest goal;
  goal.marker_id = kGoal;
  machine.start_goal(goal, 0.0);

  RunResult r;
  r.start = start;
  double now = 0.0;
  Pose2D at_vanish = start;
  bool vanish_seen = false;
  StepOutput out;
  while (now < max_time) {
    now += kDt;
    StepInput in;
    in.new_frame = true;
    const auto obs = sim.observe();
    if (obs) {
      in.markers.push_back(MarkerObservation{kGoal, *obs, now});
    }
    out = machine.step(in, now);
    if (out.state == State::APPROACH || out.state == State::FINAL || out.state == State::RECOVER) {
      ++r.approach_steps;
      if (obs) {
        ++r.approach_steps_in_view;
      }
    }
    r.entered_recover = r.entered_recover || out.state == State::RECOVER;
    r.retries = out.retries;
    if (record) {
      r.trajectory.push_back({now, sim.robot().x, sim.robot().y, sim.robot().yaw,
          static_cast<int>(out.state)});
    }
    if (out.finished) {
      break;
    }
    sim.step(out.cmd, kDt);
    if (camera.vanish_time >= 0.0 && !vanish_seen && sim.time() >= camera.vanish_time) {
      vanish_seen = true;
      at_vanish = sim.robot();
    }
    const auto e = sim.true_errors();
    if (e.d - params.control.front_offset < 0.0) {
      r.crossed_plane = true;
    }
  }
  r.finished = out.finished;
  r.duration = now;
  r.outcome = machine.result().outcome;
  const auto e = sim.true_errors();
  r.final_gap = e.d - params.control.front_offset;
  r.final_lateral = e.e_y;
  r.final_heading = e.psi;
  r.final_speed = std::abs(out.cmd.v);
  r.docked_in_tolerance = r.finished && r.outcome == Outcome::DOCKED &&
    std::abs(r.final_gap - 0.10) <= 0.02 && std::abs(e.e_y) <= 0.03 &&
    std::abs(e.psi) <= 5.0 * M_PI / 180.0;
  if (vanish_seen) {
    r.travel_after_vanish = std::hypot(sim.robot().x - at_vanish.x, sim.robot().y - at_vanish.y);
  }
  return r;
}

/// A random start: range 1 to 4 m, up to 50 degrees off the normal, heading up to 30 degrees off.
Pose2D random_start(std::mt19937 & rng)
{
  std::uniform_real_distribution<double> range(1.0, 4.0);
  std::uniform_real_distribution<double> offset(-50.0 * M_PI / 180.0, 50.0 * M_PI / 180.0);
  std::uniform_real_distribution<double> heading(-30.0 * M_PI / 180.0, 30.0 * M_PI / 180.0);
  const double r = range(rng);
  const double a = offset(rng);
  Pose2D p;
  p.x = r * std::cos(a);
  p.y = r * std::sin(a);
  p.yaw = std::atan2(-p.y, -p.x) + heading(rng);
  return p;
}

/// Write one trajectory to a CSV file in the directory named by DOCK_CSV_DIR, if it is set.
void maybe_write_csv(const std::string & name, const RunResult & r)
{
  const char * dir = std::getenv("DOCK_CSV_DIR");
  if (dir == nullptr) {
    return;
  }
  std::ofstream f(std::string(dir) + "/" + name + ".csv");
  f << "t,x,y,yaw,state\n";
  for (const auto & s : r.trajectory) {
    f << s.t << "," << s.x << "," << s.y << "," << s.yaw << "," << s.state << "\n";
  }
}

struct Batch
{
  int total = 0;
  int docked = 0;
  int crossed = 0;
  int approach_steps = 0;
  int in_view = 0;
  int recovered = 0;
  double time_sum = 0.0;
  double time_max = 0.0;
};

/// Run many random starts and gather the pass counts; prints the failed starts for debugging.
Batch run_batch(int count, const CameraModel & camera_template, unsigned seed, bool csv)
{
  std::mt19937 rng(seed);
  Batch b;
  for (int i = 0; i < count; ++i) {
    const Pose2D start = random_start(rng);
    CameraModel cam = camera_template;
    const RunResult r = run(start, cam, seed * 1000 + i, 150.0, MachineParams(), csv && i < 10);
    ++b.total;
    b.docked += r.docked_in_tolerance ? 1 : 0;
    b.crossed += r.crossed_plane ? 1 : 0;
    b.recovered += r.entered_recover ? 1 : 0;
    b.time_sum += r.duration;
    b.time_max = std::max(b.time_max, r.duration);
    b.approach_steps += r.approach_steps;
    b.in_view += r.approach_steps_in_view;
    if (csv && i < 10) {
      maybe_write_csv("cl1_" + std::to_string(i), r);
    }
    if (!r.docked_in_tolerance) {
      std::printf(
        "  FAIL #%d start (%.2f, %.2f, yaw %.2f) finished=%d outcome=%d gap=%.3f lat=%.3f "
        "psi=%.3f recover=%d retries=%d t=%.1f\n",
        i, start.x, start.y, start.yaw, r.finished, static_cast<int>(r.outcome), r.final_gap,
        r.final_lateral, r.final_heading, r.entered_recover, r.retries, r.duration);
    }
  }
  return b;
}

}  // namespace

// CL1: random starts all over the plausible range end docked within tolerance.
TEST(ClosedLoop, CL1_RandomStarts)
{
  const Batch b = run_batch(500, CameraModel{}, 1, true);
  std::printf(
    "CL1: docked %d / %d, crossed %d, recovered %d, mean t %.1f max t %.1f\n", b.docked, b.total,
    b.crossed, b.recovered, b.time_sum / b.total, b.time_max);
  EXPECT_GE(b.docked, 0.98 * b.total);
  EXPECT_EQ(b.crossed, 0);
  // CL6: the marker stays in view for nearly all of the approach.
  const double fraction = static_cast<double>(b.in_view) / std::max(b.approach_steps, 1);
  std::printf("CL6: marker in view %.4f of approach time\n", fraction);
  EXPECT_GE(fraction, 0.99);
}

// CL2: the same with half-pixel corner noise and 10% of frames dropped.
TEST(ClosedLoop, CL2_NoiseAndDrops)
{
  CameraModel cam;
  cam.pixel_noise = 0.5;
  cam.drop_probability = 0.1;
  const Batch b = run_batch(500, cam, 2, false);
  std::printf("CL2: docked %d / %d, crossed %d\n", b.docked, b.total, b.crossed);
  EXPECT_GE(b.docked, 0.95 * b.total);
  EXPECT_EQ(b.crossed, 0);
}

// CL3: with the marker never visible the goal fails with TARGET_NOT_FOUND and the robot is still.
TEST(ClosedLoop, CL3_MarkerNeverVisible)
{
  CameraModel cam;
  cam.vanish_time = 0.0;
  const RunResult r = run(Pose2D{2.0, 0.5, 3.0}, cam, 3, 100.0);
  EXPECT_TRUE(r.finished);
  EXPECT_EQ(r.outcome, Outcome::TARGET_NOT_FOUND);
  EXPECT_DOUBLE_EQ(r.final_speed, 0.0);
}

// CL4: started close and badly offset, the gates fail, RECOVER runs, and the retry docks.
TEST(ClosedLoop, CL4_RecoverThenDock)
{
  int recovered = 0;
  int docked = 0;
  const Pose2D starts[] = {
    {0.65, 0.40, std::atan2(-0.40, -0.65)},
    {0.70, -0.35, std::atan2(0.35, -0.70) + 0.2},
    {0.80, 0.50, std::atan2(-0.50, -0.80) - 0.2}};
  for (const auto & s : starts) {
    const RunResult r = run(s, CameraModel{}, 4);
    std::printf(
      "  CL4 start (%.2f, %.2f): recover=%d retries=%d docked=%d outcome=%d gap=%.3f lat=%.3f\n",
      s.x, s.y, r.entered_recover, r.retries, r.docked_in_tolerance, static_cast<int>(r.outcome),
      r.final_gap, r.final_lateral);
    recovered += r.entered_recover ? 1 : 0;
    docked += r.docked_in_tolerance ? 1 : 0;
  }
  EXPECT_GE(recovered, 1);
  EXPECT_EQ(docked, 3);
}

// CL5: the marker vanishes for good mid-approach; the robot stops, searches, and ends FAILED.
TEST(ClosedLoop, CL5_MarkerDisappears)
{
  CameraModel cam;
  cam.vanish_time = 6.0;
  const RunResult r = run(Pose2D{3.5, 0.5, M_PI}, cam, 5, 120.0);
  EXPECT_TRUE(r.finished);
  EXPECT_EQ(r.outcome, Outcome::TARGET_LOST);
  EXPECT_DOUBLE_EQ(r.final_speed, 0.0);
  EXPECT_FALSE(r.crossed_plane);
  // It may coast while the command decays (hold time and braking), but must not drive blind.
  EXPECT_LT(r.travel_after_vanish, 0.5);
}
