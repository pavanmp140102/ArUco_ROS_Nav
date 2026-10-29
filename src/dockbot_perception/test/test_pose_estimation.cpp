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
// @Date: 2026-10-12
// @Description: Unit tests P1 to P8 for the ROS-free ArUco pose estimation library.

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "dockbot_perception/pose_estimation.hpp"
#include "synthetic_marker.hpp"

using dockbot_perception::Detection;
using dockbot_perception::detect_markers;
using dockbot_perception::estimate_marker_pose;
using dockbot_perception::get_dictionary;
using dockbot_perception::testing::dockbot_camera;
using dockbot_perception::testing::draw_synthetic_marker;
using dockbot_perception::testing::marker_pose;
using dockbot_perception::testing::project_corners;

namespace
{

constexpr double kMarkerSize = 0.15;
constexpr double kPi = 3.14159265358979323846;
const std::vector<double> kNoDistortion;

/// Plain grey background, like the walls of the simulated room.
cv::Mat blank_image()
{
  return cv::Mat(720, 1280, CV_8UC1, cv::Scalar(140));
}

/// Angle in degrees between the normals (marker z axes) of two poses.
double normal_error_deg(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  const double c = a.linear().col(2).dot(b.linear().col(2));
  return std::acos(std::max(-1.0, std::min(1.0, c))) * 180.0 / kPi;
}

}  // namespace

// P1: range and yaw sweep.
TEST(PoseEstimation, P1_RangeAndYawSweep)
{
  const auto camera = dockbot_camera();
  const auto dict = get_dictionary("DICT_4X4_50");
  const std::vector<double> ranges = {0.30, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0};
  for (const double range : ranges) {
    for (int yaw_deg = -60; yaw_deg <= 60; yaw_deg += 20) {
      const double yaw = yaw_deg * kPi / 180.0;
      const auto truth = marker_pose(0.0, 0.0, range, yaw);
      cv::Mat image = blank_image();
      draw_synthetic_marker(image, camera, dict, 4, 7, kMarkerSize, truth);

      const auto found = detect_markers(image, dict);
      const std::string where =
        "range " + std::to_string(range) + " yaw " + std::to_string(yaw_deg);
      ASSERT_EQ(found.size(), 1u) << where;
      EXPECT_EQ(found[0].id, 7) << where;
      const auto pose =
        estimate_marker_pose(found[0].corners, kMarkerSize, camera.K, kNoDistortion);
      ASSERT_TRUE(pose.has_value()) << where;

      const double position_error =
        (pose->T_optical_marker.translation() - truth.translation()).norm();
      EXPECT_LE(position_error, std::max(0.02 * range, 0.01)) << where;
      if (range < 2.0) {
        EXPECT_LE(normal_error_deg(pose->T_optical_marker, truth), 3.0) << where;
      }
    }
  }
}

// P2: nothing to find.
TEST(PoseEstimation, P2_NoMarker)
{
  const auto dict = get_dictionary("DICT_4X4_50");
  cv::Mat image = blank_image();
  cv::rectangle(image, cv::Rect(200, 200, 300, 200), cv::Scalar(30), -1);
  cv::rectangle(image, cv::Rect(700, 100, 100, 400), cv::Scalar(220), 5);
  cv::circle(image, cv::Point(900, 500), 80, cv::Scalar(10), -1);
  std::vector<Detection> found;
  EXPECT_NO_THROW(found = detect_markers(image, dict));
  EXPECT_TRUE(found.empty());
  EXPECT_NO_THROW(found = detect_markers(blank_image(), dict));
  EXPECT_TRUE(found.empty());
  EXPECT_NO_THROW(found = detect_markers(cv::Mat(), dict));
  EXPECT_TRUE(found.empty());
}

// P3: several markers in one frame.
TEST(PoseEstimation, P3_ThreeMarkers)
{
  const auto camera = dockbot_camera();
  const auto dict = get_dictionary("DICT_4X4_50");
  const std::vector<std::pair<int, Eigen::Isometry3d>> markers = {
    {0, marker_pose(-0.6, 0.0, 2.0, 0.3)},
    {1, marker_pose(0.0, 0.1, 1.2, -0.2)},
    {4, marker_pose(0.7, -0.1, 1.8, 0.5)},
  };
  cv::Mat image = blank_image();
  for (const auto & [id, T] : markers) {
    draw_synthetic_marker(image, camera, dict, 4, id, kMarkerSize, T);
  }
  const auto found = detect_markers(image, dict);
  ASSERT_EQ(found.size(), 3u);
  for (const auto & [id, truth] : markers) {
    bool seen = false;
    for (const auto & d : found) {
      if (d.id != id) {
        continue;
      }
      seen = true;
      const auto pose = estimate_marker_pose(d.corners, kMarkerSize, camera.K, kNoDistortion);
      ASSERT_TRUE(pose.has_value()) << "id " << id;
      const double range = truth.translation().norm();
      EXPECT_LE(
        (pose->T_optical_marker.translation() - truth.translation()).norm(),
        std::max(0.02 * range, 0.01)) << "id " << id;
    }
    EXPECT_TRUE(seen) << "id " << id;
  }
}

// P4: a marker from another dictionary, and a mirrored one, are not reported.
TEST(PoseEstimation, P4_WrongDictionaryAndMirrored)
{
  const auto camera = dockbot_camera();
  const auto dict4 = get_dictionary("DICT_4X4_50");
  const auto dict5 = get_dictionary("DICT_5X5_50");
  const auto truth = marker_pose(0.0, 0.0, 1.0, 0.2);

  cv::Mat other = blank_image();
  draw_synthetic_marker(other, camera, dict5, 5, 3, kMarkerSize, truth);
  EXPECT_TRUE(detect_markers(other, dict4).empty());

  // Marker 7 is not left-right symmetric, so its mirror is not a valid code.
  cv::Mat mirrored = blank_image();
  draw_synthetic_marker(mirrored, camera, dict4, 4, 7, kMarkerSize, truth, true);
  EXPECT_TRUE(detect_markers(mirrored, dict4).empty());
}

// P5: partial markers are not reported.
TEST(PoseEstimation, P5_CutOffAndCovered)
{
  const auto camera = dockbot_camera();
  const auto dict = get_dictionary("DICT_4X4_50");

  // Half of the marker lies outside the image.
  cv::Mat cut = blank_image();
  draw_synthetic_marker(cut, camera, dict, 4, 2, kMarkerSize, marker_pose(-0.65, 0.0, 1.0, 0.0));
  EXPECT_TRUE(detect_markers(cut, dict).empty());

  // One corner covered by an object.
  const auto truth = marker_pose(0.0, 0.0, 1.0, 0.0);
  cv::Mat covered = blank_image();
  draw_synthetic_marker(covered, camera, dict, 4, 2, kMarkerSize, truth);
  const auto corners = project_corners(camera, kMarkerSize, truth);
  cv::circle(covered, corners[0], 30, cv::Scalar(140), -1);  // An object the colour of the wall.
  const auto found = detect_markers(covered, dict);
  for (const auto & d : found) {
    // Any marker that is reported must still have a sane pose.
    const auto pose = estimate_marker_pose(d.corners, kMarkerSize, camera.K, kNoDistortion);
    if (pose) {
      EXPECT_LT((pose->T_optical_marker.translation() - truth.translation()).norm(), 0.05);
    }
  }
  EXPECT_TRUE(found.empty());
}

// P6: optical frame to base_link with the planned camera mount.
TEST(PoseEstimation, P6_OpticalToBaseLink)
{
  // Mount from the robot description: 0.05 m behind base_link, optical rpy = (-pi/2, 0, -pi/2).
  Eigen::Isometry3d T_base_optical = Eigen::Isometry3d::Identity();
  T_base_optical.linear() =
    (Eigen::AngleAxisd(-kPi / 2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-kPi / 2, Eigen::Vector3d::UnitX())).toRotationMatrix();
  T_base_optical.translation() = Eigen::Vector3d(-0.05, 0.0, 0.20);

  const auto camera = dockbot_camera();
  const auto dict = get_dictionary("DICT_4X4_50");
  const auto T_optical_marker = marker_pose(0.0, 0.0, 1.0, 0.0);

  // Through the whole chain: render, detect, solve, transform.
  cv::Mat image = blank_image();
  draw_synthetic_marker(image, camera, dict, 4, 2, kMarkerSize, T_optical_marker);
  const auto found = detect_markers(image, dict);
  ASSERT_EQ(found.size(), 1u);
  const auto pose = estimate_marker_pose(found[0].corners, kMarkerSize, camera.K, kNoDistortion);
  ASSERT_TRUE(pose.has_value());

  const auto T_base_marker =
    dockbot_perception::transform_pose(pose->T_optical_marker, T_base_optical);
  EXPECT_NEAR(T_base_marker.translation().x(), 0.95, 0.01);
  EXPECT_NEAR(T_base_marker.translation().y(), 0.0, 0.01);
  const Eigen::Vector3d normal = T_base_marker.linear().col(2);
  EXPECT_NEAR(normal.x(), -1.0, 0.02);  // Points back at the robot.
  EXPECT_NEAR(normal.y(), 0.0, 0.02);

  const auto rb = dockbot_perception::range_and_bearing(T_base_marker.translation());
  EXPECT_NEAR(rb.range, 0.95, 0.01);
  EXPECT_NEAR(rb.bearing, 0.0, 0.01);

  // A marker to the left of the robot has a positive bearing.
  const auto left = dockbot_perception::transform_pose(
    marker_pose(-0.5, 0.0, 1.0, 0.0), T_base_optical);
  EXPECT_GT(dockbot_perception::range_and_bearing(left.translation()).bearing, 0.0);
}

// P7: the detection wrapper returns IDs and corners in the documented order.
TEST(PoseEstimation, P7_DetectionShape)
{
  const auto camera = dockbot_camera();
  const auto dict = get_dictionary("DICT_4X4_50");
  const auto truth = marker_pose(0.1, 0.05, 1.0, 0.3);
  cv::Mat image = blank_image();
  draw_synthetic_marker(image, camera, dict, 4, 3, kMarkerSize, truth);
  const auto found = detect_markers(image, dict);
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0].id, 3);
  // Top-left, top-right, bottom-right, bottom-left, in pixels.
  const auto expected = project_corners(camera, kMarkerSize, truth);
  for (size_t k = 0; k < 4; ++k) {
    EXPECT_LT(cv::norm(found[0].corners[k] - expected[k]), 1.0) << "corner " << k;
  }

  EXPECT_THROW(get_dictionary("DICT_NOPE"), std::invalid_argument);
  EXPECT_THROW(detect_markers(cv::Mat(10, 10, CV_8UC3), dict), std::invalid_argument);
}

// P8: with noisy corners the near face-on ambiguity never yields a normal pointing away.
TEST(PoseEstimation, P8_AmbiguityHandling)
{
  const auto camera = dockbot_camera();
  const auto truth = marker_pose(0.0, 0.0, 3.0, 5.0 * kPi / 180.0);
  const auto clean = project_corners(camera, kMarkerSize, truth);
  std::mt19937 rng(12345);
  std::normal_distribution<float> noise(0.0f, 0.5f);

  int trials = 0;
  for (int i = 0; i < 300; ++i) {
    auto corners = clean;
    for (auto & c : corners) {
      c += cv::Point2f(noise(rng), noise(rng));
    }
    const auto pose = estimate_marker_pose(corners, kMarkerSize, camera.K, kNoDistortion);
    ASSERT_TRUE(pose.has_value());
    ++trials;
    const Eigen::Vector3d normal = pose->T_optical_marker.linear().col(2);
    // The normal must point toward the camera, never away from it.
    EXPECT_LT(normal.dot(pose->T_optical_marker.translation()), 0.0) << "trial " << i;
    EXPECT_LT(
      (pose->T_optical_marker.translation() - truth.translation()).norm(), 0.04 * 3.0)
      << "trial " << i;
    EXPECT_LT(normal_error_deg(pose->T_optical_marker, truth), 25.0) << "trial " << i;
  }
  EXPECT_EQ(trials, 300);

  // With exact corners the lower-error solution is the true one.
  const auto exact = estimate_marker_pose(clean, kMarkerSize, camera.K, kNoDistortion);
  ASSERT_TRUE(exact.has_value());
  EXPECT_LT(exact->reprojection_error, 0.01);
  EXPECT_LT(normal_error_deg(exact->T_optical_marker, truth), 0.5);
}

// Debug drawing leaves a valid image and rejects the wrong type.
TEST(PoseEstimation, DrawDebug)
{
  const auto camera = dockbot_camera();
  const auto truth = marker_pose(0.0, 0.0, 1.0, 0.0);
  const auto corners = project_corners(camera, kMarkerSize, truth);
  dockbot_perception::MarkerObservation obs{
    {2, corners}, {truth, 0.0}};
  cv::Mat color(720, 1280, CV_8UC3, cv::Scalar(140, 140, 140));
  EXPECT_NO_THROW(
    dockbot_perception::draw_debug(color, {obs}, camera.K, kNoDistortion, kMarkerSize));
  EXPECT_GT(cv::countNonZero(color.reshape(1) != 140), 0);
  cv::Mat gray = blank_image();
  EXPECT_THROW(
    dockbot_perception::draw_debug(gray, {obs}, camera.K, kNoDistortion, kMarkerSize),
    std::invalid_argument);
}
