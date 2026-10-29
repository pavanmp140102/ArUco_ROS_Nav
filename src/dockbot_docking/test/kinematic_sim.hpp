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
// @Description: Test-only unicycle and camera model for closed-loop docking tests without ROS.

#ifndef KINEMATIC_SIM_HPP_
#define KINEMATIC_SIM_HPP_

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <optional>
#include <random>

#include "dockbot_docking/control_law.hpp"

namespace dockbot_docking
{
namespace testing
{

/// Planar pose of the robot (base_link) in the world.
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/// Camera and noise settings of the simulated sensor.
struct CameraModel
{
  /// Camera position in base_link along x: 0.20 m behind the front face at 0.15 m.
  double x_offset = -0.05;
  double half_fov = 0.543;  // 31.1 degrees
  double marker_size = 0.15;
  double min_range = 0.22;
  double max_range = 5.0;
  /// Largest angle between the marker normal and the line of sight, radians.
  double max_view_angle = 1.22;  // 70 degrees
  /// Corner noise in pixels (0 disables noise).
  double pixel_noise = 0.0;
  double focal_px = 1060.0;
  /// Probability that a frame is dropped.
  double drop_probability = 0.0;
  /// Time after which the marker is never seen again (negative: never).
  double vanish_time = -1.0;
};

/**
 * Unicycle robot plus a pinhole-camera view of one marker, with seeded noise.
 *
 * Lets the controller be run in closed loop thousands of times in seconds, with no simulator.
 */
class KinematicSim
{
public:
  /// Create a sim with a fixed seed; the marker sits at marker_xy with its normal at marker_yaw.
  KinematicSim(
    const Pose2D & robot, const Eigen::Vector2d & marker_xy, double marker_yaw,
    const CameraModel & camera, unsigned seed)
  : robot_(robot), marker_xy_(marker_xy), marker_yaw_(marker_yaw), camera_(camera), rng_(seed) {}

  /// Integrate the commanded velocities for dt seconds (exact arc for a unicycle).
  void step(const VelocityCommand & cmd, double dt)
  {
    time_ += dt;
    if (std::abs(cmd.w) < 1e-9) {
      robot_.x += cmd.v * dt * std::cos(robot_.yaw);
      robot_.y += cmd.v * dt * std::sin(robot_.yaw);
    } else {
      const double yaw1 = robot_.yaw + cmd.w * dt;
      robot_.x += cmd.v / cmd.w * (std::sin(yaw1) - std::sin(robot_.yaw));
      robot_.y -= cmd.v / cmd.w * (std::cos(yaw1) - std::cos(robot_.yaw));
      robot_.yaw = yaw1;
    }
  }

  /// Marker pose in base_link, or nothing if it is outside the view or the frame is dropped.
  std::optional<Eigen::Isometry3d> observe()
  {
    if (camera_.vanish_time >= 0.0 && time_ >= camera_.vanish_time) {
      return std::nullopt;
    }
    if (camera_.drop_probability > 0.0 && uniform_(rng_) < camera_.drop_probability) {
      return std::nullopt;
    }
    const Eigen::Isometry3d T = true_pose();
    const Eigen::Vector2d p = T.translation().head<2>() - Eigen::Vector2d(camera_.x_offset, 0.0);
    const double dist = p.norm();
    if (dist < camera_.min_range || dist > camera_.max_range) {
      return std::nullopt;
    }
    const double half_extent = std::atan(0.5 * camera_.marker_size / dist);
    if (std::abs(std::atan2(p.y(), p.x())) + half_extent > camera_.half_fov) {
      return std::nullopt;
    }
    const Eigen::Vector3d n = T.linear().col(2);
    const double cos_view = n.head<2>().dot(-p) / dist;  // normal against marker-to-camera
    if (std::acos(std::clamp(cos_view, -1.0, 1.0)) > camera_.max_view_angle) {
      return std::nullopt;
    }
    if (camera_.pixel_noise <= 0.0) {
      return T;
    }
    return noisy(T, dist);
  }

  /// Exact marker pose in base_link (marker z axis out of the printed face).
  Eigen::Isometry3d true_pose() const
  {
    const Eigen::Vector2d w = marker_xy_ - Eigen::Vector2d(robot_.x, robot_.y);
    const Eigen::Rotation2Dd Rinv(-robot_.yaw);
    const Eigen::Vector2d p = Rinv * w;
    const double normal_yaw = marker_yaw_ - robot_.yaw;
    Eigen::Matrix3d R0;
    R0.col(0) = Eigen::Vector3d(0, 1, 0);  // marker x: its right when seen from the front
    R0.col(1) = Eigen::Vector3d(0, 0, 1);  // marker y: up
    R0.col(2) = Eigen::Vector3d(1, 0, 0);  // marker z: out of the face
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::AngleAxisd(normal_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R0;
    T.translation() = Eigen::Vector3d(p.x(), p.y(), 0.0);
    return T;
  }

  /// True errors (d, e_y, psi, bearing, range) from base_link, using the exact pose.
  DockErrors true_errors() const {return compute_errors(true_pose());}

  const Pose2D & robot() const {return robot_;}
  double time() const {return time_;}

private:
  /// Perturb the pose the way corner noise would: range worst, lateral and tilt smaller.
  Eigen::Isometry3d noisy(const Eigen::Isometry3d & T, double dist)
  {
    std::normal_distribution<double> g(0.0, 1.0);
    const double s = camera_.pixel_noise;
    const double f = camera_.focal_px;
    const double sigma_lateral = s * dist / f;
    const double sigma_range = s * dist * dist / (f * camera_.marker_size);
    const double sigma_yaw = 2.0 * s * dist / (f * camera_.marker_size);
    Eigen::Isometry3d out = T;
    out.translation() += Eigen::Vector3d(
      sigma_range * g(rng_), sigma_lateral * g(rng_), 0.0);
    out.linear() = Eigen::AngleAxisd(sigma_yaw * g(rng_), Eigen::Vector3d::UnitZ()) * T.linear();
    return out;
  }

  Pose2D robot_;
  Eigen::Vector2d marker_xy_;
  double marker_yaw_;
  CameraModel camera_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  double time_ = 0.0;
};

}  // namespace testing
}  // namespace dockbot_docking

#endif  // KINEMATIC_SIM_HPP_
