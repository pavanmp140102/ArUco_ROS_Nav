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
// @Description: Test helper that renders an ArUco marker into an image at a known pose.

#ifndef SYNTHETIC_MARKER_HPP_
#define SYNTHETIC_MARKER_HPP_

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include "dockbot_perception/pose_estimation.hpp"

namespace dockbot_perception
{
namespace testing
{

/// Pinhole camera with no distortion, used to render and then re-measure synthetic markers.
struct SyntheticCamera
{
  cv::Matx33d K;
  cv::Size size;
};

/// The simulated dockbot camera: 1280x720 with a horizontal field of view of 1.0856 rad.
inline SyntheticCamera dockbot_camera()
{
  const double fx = 640.0 / std::tan(1.0856 / 2.0);
  return {cv::Matx33d(fx, 0, 640.0, 0, fx, 360.0, 0, 0, 1), cv::Size(1280, 720)};
}

/**
 * Marker pose in the optical frame at lateral offset x, height y (down), and depth z, turned by
 * yaw about the vertical axis. Zero yaw is face-on: the marker's z axis points back at the camera.
 */
inline Eigen::Isometry3d marker_pose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Matrix3d face_on = Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()).toRotationMatrix() * face_on;
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

/// The marker's corners projected into the image, in the order of Detection::corners.
inline std::array<cv::Point2f, 4> project_corners(
  const SyntheticCamera & camera, double marker_size, const Eigen::Isometry3d & T)
{
  const float h = static_cast<float>(marker_size / 2.0);
  const std::vector<cv::Point3f> object = {{-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};
  cv::Mat R;
  cv::eigen2cv(Eigen::Matrix3d(T.linear()), R);
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  const cv::Vec3d tvec(T.translation().x(), T.translation().y(), T.translation().z());
  std::vector<cv::Point2f> pts;
  cv::projectPoints(object, rvec, tvec, camera.K, std::vector<double>(), pts);
  return {pts[0], pts[1], pts[2], pts[3]};
}

/**
 * Draw one marker into a greyscale image as the camera would see it.
 *
 * @param bits marker grid size, e.g. 4 for DICT_4X4_50 (the black border adds two cells).
 * @param mirrored draw the marker flipped, which is not a valid code.
 * The scene is rendered at twice the resolution and averaged down so edges are anti-aliased.
 */
inline void draw_synthetic_marker(
  cv::Mat & image, const SyntheticCamera & camera, const Dictionary & dictionary, int bits,
  int id, double marker_size, const Eigen::Isometry3d & T, bool mirrored = false)
{
  const int cells = bits + 2;
  const int cell_px = 16;
  cv::Mat code;
#if DOCKBOT_OPENCV_NEW_ARUCO
  cv::aruco::generateImageMarker(dictionary, id, cells * cell_px, code, 1);
#else
  cv::aruco::drawMarker(dictionary, id, cells * cell_px, code, 1);
#endif
  if (mirrored) {
    cv::flip(code, code, 1);
  }
  // A white margin of one cell around the code, as a printed marker has.
  cv::Mat padded;
  cv::copyMakeBorder(code, padded, cell_px, cell_px, cell_px, cell_px, cv::BORDER_CONSTANT, 255);

  const double cell = marker_size / cells;
  const double half = marker_size / 2.0 + cell;
  const std::array<cv::Point2f, 4> quad =
    project_corners(camera, 2.0 * half, T);

  // Supersampled canvas: pixel centres of the small image sit at (2u + 0.5) on the large one.
  cv::Mat canvas;
  cv::resize(image, canvas, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
  std::vector<cv::Point2f> dst;
  for (const auto & p : quad) {
    dst.emplace_back(2.0f * p.x + 0.5f, 2.0f * p.y + 0.5f);
  }
  // Pixel centres sit at integer coordinates, so the outer edge of the first pixel is at -0.5.
  const float s = static_cast<float>(padded.cols) - 0.5f;
  const std::vector<cv::Point2f> src = {{-0.5f, -0.5f}, {s, -0.5f}, {s, s}, {-0.5f, s}};
  const cv::Mat H = cv::getPerspectiveTransform(src, dst);
  cv::Mat warped;
  cv::Mat mask;
  cv::warpPerspective(padded, warped, H, canvas.size(), cv::INTER_LINEAR);
  cv::warpPerspective(
    cv::Mat(padded.size(), CV_8UC1, cv::Scalar(255)), mask, H, canvas.size(), cv::INTER_NEAREST);
  warped.copyTo(canvas, mask);
  cv::resize(canvas, image, image.size(), 0, 0, cv::INTER_AREA);
}

}  // namespace testing
}  // namespace dockbot_perception

#endif  // SYNTHETIC_MARKER_HPP_
