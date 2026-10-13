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
// @Description: ROS-free ArUco detection and pose maths: grey image to marker poses in base_link.

#include "dockbot_perception/pose_estimation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace dockbot_perception
{

namespace
{

/// Dictionary names accepted by get_dictionary, with their OpenCV identifiers.
const std::vector<std::pair<const char *, int>> & dictionary_table()
{
  static const std::vector<std::pair<const char *, int>> table = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };
  return table;
}

/// Convert an OpenCV rotation vector and translation to a rigid transform, for the pose maths.
Eigen::Isometry3d to_isometry(const cv::Mat & rvec, const cv::Mat & tvec)
{
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d Re;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Re(r, c) = R.at<double>(r, c);
    }
  }
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Re;
  T.translation() = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  return T;
}

/**
 * Marker corner positions in a frame with x right, y DOWN, z into the face.
 *
 * With y up (OpenCV's usual marker frame) a marker seen face-on is a rotation of exactly pi
 * from the camera frame, where rotation vectors are singular and the solver returns zero.
 * With y down the same view is the identity, so the maths stays well behaved at the moment the
 * robot is docked. The result is flipped to the documented convention afterwards.
 */
std::vector<cv::Point3f> marker_object_points(double marker_size)
{
  const float h = static_cast<float>(marker_size / 2.0);
  return {{-h, -h, 0.0f}, {h, -h, 0.0f}, {h, h, 0.0f}, {-h, h, 0.0f}};
}

}  // namespace

Dictionary get_dictionary(const std::string & name)
{
  for (const auto & [key, id] : dictionary_table()) {
    if (name == key) {
#if DOCKBOT_OPENCV_NEW_ARUCO
      return cv::aruco::getPredefinedDictionary(id);
#else
      return cv::aruco::getPredefinedDictionary(
          static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(id));
#endif
    }
  }
  throw std::invalid_argument("Unknown ArUco dictionary '" + name + "'; expected e.g. DICT_4X4_50");
}

std::vector<Detection> detect_markers(const cv::Mat & gray, const Dictionary & dictionary)
{
  if (gray.type() != CV_8UC1) {
    throw std::invalid_argument("detect_markers needs an 8-bit single-channel image");
  }
  std::vector<Detection> result;
  if (gray.empty()) {
    return result;
  }

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
#if DOCKBOT_OPENCV_NEW_ARUCO
  cv::aruco::DetectorParameters params;
  params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
  cv::aruco::ArucoDetector detector(dictionary, params);
  detector.detectMarkers(gray, corners, ids);
  const int grid = dictionary.markerSize + 2;
#else
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
  cv::aruco::detectMarkers(gray, dictionary, corners, ids, params);
  const int grid = dictionary->markerSize + 2;
#endif

  // Refine corners to sub-pixel accuracy ourselves: OpenCV's fixed 5-pixel window is wider
  // than a marker cell at long range, which pulls corners off and biases the range by several
  // percent. Scale the window to the cell size instead.
  for (auto & marker : corners) {
    double perimeter = 0.0;
    for (size_t k = 0; k < 4; ++k) {
      perimeter += cv::norm(marker[k] - marker[(k + 1) % 4]);
    }
    const double cell_px = perimeter / 4.0 / grid;
    const int window = std::max(1, std::min(5, static_cast<int>(std::lround(cell_px * 0.4))));
    cv::cornerSubPix(
      gray, marker, cv::Size(window, window), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
  }

  for (size_t i = 0; i < ids.size(); ++i) {
    Detection d;
    d.id = ids[i];
    for (size_t k = 0; k < 4; ++k) {
      d.corners[k] = corners[i][k];
    }
    result.push_back(d);
  }
  return result;
}

std::optional<PoseEstimate> estimate_marker_pose(
  const std::array<cv::Point2f, 4> & corners, double marker_size,
  const cv::Matx33d & K, const std::vector<double> & dist)
{
  const std::vector<cv::Point3f> object = marker_object_points(marker_size);
  const std::vector<cv::Point2f> image(corners.begin(), corners.end());

  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  try {
    // IPPE_SQUARE only accepts y-up corners; IPPE is the same method for any planar target.
    cv::solvePnPGeneric(object, image, K, dist, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
  } catch (const cv::Exception &) {
    return std::nullopt;  // Degenerate corners, e.g. collinear or coincident points.
  }

  std::optional<PoseEstimate> best;
  for (size_t i = 0; i < rvecs.size(); ++i) {
    Eigen::Isometry3d T = to_isometry(rvecs[i], tvecs[i]);
    // Flip y and z: from "y down, z into the face" to "y up, z out of the face".
    T.linear() = T.linear() * Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
    if (!T.matrix().allFinite()) {
      continue;
    }
    const Eigen::Vector3d position = T.translation();
    const Eigen::Vector3d normal = T.linear().col(2);
    // A printed face can only be seen from the front: the normal must point back at the camera.
    if (position.z() <= 0.0 || normal.dot(position) >= 0.0) {
      continue;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object, rvecs[i], tvecs[i], K, dist, projected);  // Same pose, solver frame.
    double sum_sq = 0.0;
    for (size_t k = 0; k < 4; ++k) {
      const cv::Point2f d = projected[k] - image[k];
      sum_sq += static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y;
    }
    const double error = std::sqrt(sum_sq / 4.0);
    if (!std::isfinite(error)) {
      continue;
    }
    if (!best || error < best->reprojection_error) {
      best = PoseEstimate{T, error};
    }
  }
  return best;
}

Eigen::Isometry3d transform_pose(
  const Eigen::Isometry3d & T_optical_marker, const Eigen::Isometry3d & T_base_optical)
{
  return T_base_optical * T_optical_marker;
}

RangeBearing range_and_bearing(const Eigen::Vector3d & position)
{
  return {std::hypot(position.x(), position.y()), std::atan2(position.y(), position.x())};
}

void draw_debug(
  cv::Mat & image, const std::vector<MarkerObservation> & observations,
  const cv::Matx33d & K, const std::vector<double> & dist, double marker_size)
{
  if (image.type() != CV_8UC3) {
    throw std::invalid_argument("draw_debug needs an 8-bit three-channel image");
  }
  for (const auto & obs : observations) {
    std::vector<cv::Point> outline;
    for (const auto & p : obs.detection.corners) {
      outline.emplace_back(cvRound(p.x), cvRound(p.y));
    }
    cv::polylines(image, outline, true, cv::Scalar(0, 255, 0), 2);
    cv::circle(image, outline[0], 5, cv::Scalar(0, 0, 255), -1);  // Marks the top-left corner.
    cv::putText(
      image, std::to_string(obs.detection.id), outline[0] + cv::Point(6, -6),
      cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);

    cv::Mat R;
    cv::eigen2cv(Eigen::Matrix3d(obs.pose.T_optical_marker.linear()), R);
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    const Eigen::Vector3d t = obs.pose.T_optical_marker.translation();
    cv::drawFrameAxes(
      image, K, dist, rvec, cv::Vec3d(t.x(), t.y(), t.z()), static_cast<float>(marker_size * 0.75));
  }
}

}  // namespace dockbot_perception
