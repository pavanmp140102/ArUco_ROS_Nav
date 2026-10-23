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
// @Description: Unit tests C1 to C4 and C8 for the docking control law library.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "dockbot_docking/control_law.hpp"
#include "kinematic_sim.hpp"

using dockbot_docking::compute_errors;
using dockbot_docking::ControlParams;
using dockbot_docking::DockErrors;
using dockbot_docking::VelocityCommand;
using dockbot_docking::testing::CameraModel;
using dockbot_docking::testing::KinematicSim;
using dockbot_docking::testing::Pose2D;

namespace
{

/// Marker pose in base_link for a marker at (x, y) whose normal points along normal_yaw.
Eigen::Isometry3d marker_pose(double x, double y, double normal_yaw)
{
  KinematicSim sim(Pose2D{}, Eigen::Vector2d(x, y), normal_yaw, CameraModel{}, 1);
  return sim.true_pose();
}

double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

}  // namespace

// C1: pins the sign conventions of d, e_y, psi and bearing against hand-computed values.
TEST(ControlLaw, ErrorsDeadAheadAndSquare)
{
  const DockErrors e = compute_errors(marker_pose(2.0, 0.0, M_PI));
  EXPECT_NEAR(e.d, 2.0, 1e-9);
  EXPECT_NEAR(e.e_y, 0.0, 1e-9);
  EXPECT_NEAR(e.psi, 0.0, 1e-9);
  EXPECT_NEAR(e.bearing, 0.0, 1e-9);
  EXPECT_NEAR(e.range, 2.0, 1e-9);
}

TEST(ControlLaw, ErrorsMarkerToTheLeft)
{
  // Marker 0.5 m to the robot's left; the robot is to the right of the marker's normal line.
  const DockErrors e = compute_errors(marker_pose(2.0, 0.5, M_PI));
  EXPECT_NEAR(e.d, 2.0, 1e-9);
  EXPECT_NEAR(e.e_y, 0.5, 1e-9);
  EXPECT_NEAR(e.psi, 0.0, 1e-9);
  EXPECT_NEAR(e.bearing, std::atan2(0.5, 2.0), 1e-9);
  EXPECT_NEAR(e.range, std::hypot(2.0, 0.5), 1e-9);
}

TEST(ControlLaw, ErrorsMarkerToTheRight)
{
  const DockErrors e = compute_errors(marker_pose(2.0, -0.5, M_PI));
  EXPECT_NEAR(e.e_y, -0.5, 1e-9);
  EXPECT_NEAR(e.bearing, -std::atan2(0.5, 2.0), 1e-9);
}

TEST(ControlLaw, ErrorsRotatedMarker)
{
  // Normal turned 0.3 rad counter-clockwise from square: the inward normal is to the robot's left.
  const DockErrors e = compute_errors(marker_pose(2.0, 0.0, M_PI + 0.3));
  EXPECT_NEAR(e.psi, 0.3, 1e-9);
  EXPECT_NEAR(e.d, 2.0 * std::cos(0.3), 1e-9);
  EXPECT_NEAR(e.e_y, -2.0 * std::sin(0.3), 1e-9);
  EXPECT_NEAR(e.bearing, 0.0, 1e-9);
}

// C2: the stop condition fires exactly when d - front_offset <= dock_distance.
TEST(ControlLaw, StopCondition)
{
  ControlParams p;
  DockErrors e;
  e.d = p.front_offset + p.dock_distance + 1e-6;
  EXPECT_FALSE(dockbot_docking::is_docked(e, p));
  e.d = p.front_offset + p.dock_distance - 1e-6;
  EXPECT_TRUE(dockbot_docking::is_docked(e, p));
  e.d = 1.0;
  EXPECT_FALSE(dockbot_docking::is_docked(e, p));
}

TEST(ControlLaw, Gates)
{
  ControlParams p;
  DockErrors e;
  EXPECT_TRUE(dockbot_docking::gates_pass(e, p));
  e.e_y = p.gate_lateral + 1e-3;
  EXPECT_FALSE(dockbot_docking::gates_pass(e, p));
  e.e_y = 0.0;
  e.psi = -(p.gate_heading + 1e-3);
  EXPECT_FALSE(dockbot_docking::gates_pass(e, p));
}

// C3: commands never exceed the speed or acceleration limits, over random inputs.
TEST(ControlLaw, LimitsHoldOverRandomInputs)
{
  const ControlParams p;
  const double dt = 0.05;
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> range(0.2, 6.0);
  std::uniform_real_distribution<double> bearing(-0.6, 0.6);
  std::uniform_real_distribution<double> psi(-M_PI, M_PI);
  std::uniform_real_distribution<double> lateral(-3.0, 3.0);
  std::uniform_real_distribution<double> dist(-0.5, 6.0);
  std::uniform_real_distribution<double> prev_v(-p.v_max, p.v_max);
  std::uniform_real_distribution<double> prev_w(-p.w_max, p.w_max);
  for (int i = 0; i < 10000; ++i) {
    DockErrors e;
    e.range = range(rng);
    e.bearing = bearing(rng);
    e.psi = psi(rng);
    e.e_y = lateral(rng);
    e.d = dist(rng);
    const VelocityCommand previous{prev_v(rng), prev_w(rng)};
    const VelocityCommand raws[] = {
      dockbot_docking::approach_command(e, p), dockbot_docking::final_command(e, p),
      dockbot_docking::recover_command(e, p), dockbot_docking::search_command(e.bearing, p)};
    for (const auto & raw : raws) {
      const VelocityCommand c = dockbot_docking::limit_command(raw, previous, dt, p);
      ASSERT_LE(std::abs(c.v), p.v_max + 1e-12);
      ASSERT_LE(std::abs(c.w), p.w_max + 1e-12);
      ASSERT_LE(std::abs(c.v - previous.v), p.a_max * dt + 1e-12);
      ASSERT_LE(std::abs(c.w - previous.w), p.alpha_max * dt + 1e-12);
    }
  }
}

// C4: the commanded heading never asks for a marker bearing beyond max_bearing_offset.
TEST(ControlLaw, FieldOfViewClamp)
{
  const ControlParams p;
  for (double range = 0.3; range <= 5.0; range += 0.25) {
    for (double bearing = -0.54; bearing <= 0.54; bearing += 0.06) {
      for (double psi = -1.4; psi <= 1.4; psi += 0.1) {
        DockErrors e;
        e.range = range;
        e.bearing = bearing;
        e.psi = psi;
        e.d = range * std::cos(psi - bearing);
        const double h = dockbot_docking::desired_heading(e, p);
        ASSERT_LE(std::abs(wrap(bearing - h)), p.max_bearing_offset + 1e-9)
          << "range " << range << " bearing " << bearing << " psi " << psi;
      }
    }
  }
}

// C8: beyond yaw_trust_range the (possibly wrong) marker orientation has no effect.
TEST(ControlLaw, FarCommandIgnoresOrientation)
{
  const ControlParams p;
  DockErrors ref;
  ref.range = 3.0;
  ref.bearing = 0.2;
  ref.psi = 0.0;
  ref.d = 3.0;
  const VelocityCommand base = dockbot_docking::approach_command(ref, p);
  for (double psi : {-2.5, -1.0, 0.7, 1.5, 3.0}) {
    DockErrors e = ref;
    e.psi = psi;
    e.d = 0.4 * ref.range;
    e.e_y = 1.7;
    const VelocityCommand c = dockbot_docking::approach_command(e, p);
    EXPECT_DOUBLE_EQ(c.v, base.v);
    EXPECT_DOUBLE_EQ(c.w, base.w);
  }
  // Pure bearing pursuit: turning toward the marker.
  EXPECT_GT(base.w, 0.0);
  EXPECT_GT(base.v, 0.0);
}

TEST(ControlLaw, SearchTurnsTowardLastBearing)
{
  const ControlParams p;
  EXPECT_GT(dockbot_docking::search_command(0.3, p).w, 0.0);
  EXPECT_LT(dockbot_docking::search_command(-0.3, p).w, 0.0);
  EXPECT_DOUBLE_EQ(dockbot_docking::search_command(0.3, p).v, 0.0);
}

TEST(ControlLaw, PoseFilterSmoothsAndResets)
{
  dockbot_docking::PoseFilter f(0.5);
  Eigen::Isometry3d a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d b = Eigen::Isometry3d::Identity();
  b.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  EXPECT_NEAR(f.update(a).translation().x(), 0.0, 1e-12);
  EXPECT_NEAR(f.update(b).translation().x(), 1.0, 1e-12);
  f.reset();
  EXPECT_FALSE(f.initialized());
  EXPECT_NEAR(f.update(b).translation().x(), 2.0, 1e-12);
}
