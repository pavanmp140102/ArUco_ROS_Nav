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

#ifndef DOCKBOT_PERCEPTION__POSE_ESTIMATION_HPP_
#define DOCKBOT_PERCEPTION__POSE_ESTIMATION_HPP_

#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace dockbot_perception
{

/// True when the installed OpenCV has the class-based ArUco API (4.7 and later).
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#define DOCKBOT_OPENCV_NEW_ARUCO 1
/// Marker dictionary type; hides the difference between the OpenCV 4.6 and 4.7+ ArUco APIs.
using Dictionary = cv::aruco::Dictionary;
#else
#define DOCKBOT_OPENCV_NEW_ARUCO 0
/// Marker dictionary type; hides the difference between the OpenCV 4.6 and 4.7+ ArUco APIs.
using Dictionary = cv::Ptr<cv::aruco::Dictionary>;
#endif

/// One marker found in an image: its ID and its four corners in pixels.
struct Detection
{
  int id;
  /// Top-left, top-right, bottom-right, bottom-left as seen with the marker upright.
  std::array<cv::Point2f, 4> corners;
};

/// A marker pose in the camera optical frame and how well it fits the observed corners.
struct PoseEstimate
{
  /// Marker frame (z out of the printed face) expressed in the camera optical frame.
  Eigen::Isometry3d T_optical_marker;
  /// RMS distance between the observed and re-projected corners, in pixels.
  double reprojection_error;
};

/// A detection together with its pose, as drawn on the debug image.
struct MarkerObservation
{
  Detection detection;
  PoseEstimate pose;
};

/// Ground-plane distance and direction from the robot to a point.
struct RangeBearing
{
  /// Distance in the x-y plane, metres.
  double range;
  /// Angle from the forward (x) axis, radians, positive to the left.
  double bearing;
};

/**
 * Look up an OpenCV ArUco dictionary by name, e.g. "DICT_4X4_50".
 *
 * The detector and the world's marker textures must use the same dictionary, so this ties
 * the configured name to the code that recognises the docking markers.
 *
 * @throws std::invalid_argument if the name is not a known dictionary.
 */
Dictionary get_dictionary(const std::string & name);

/**
 * Find every marker of the dictionary in a greyscale image, with sub-pixel corners.
 *
 * This is the first step of docking perception: it says which docking stations are in view and
 * where their corners are. The OpenCV 4.6 and 4.7+ APIs are selected at compile time.
 *
 * @param gray 8-bit single-channel image.
 * @throws std::invalid_argument if the image is not 8-bit single-channel.
 */
std::vector<Detection> detect_markers(const cv::Mat & gray, const Dictionary & dictionary);

/**
 * Solve for a marker's pose from its four corners, resolving the planar ambiguity.
 *
 * Gives the docking controller a metric position and orientation of the station from one
 * camera image. Both planar solutions are computed; solutions that put the printed face
 * pointing away from the camera, or the marker behind it, are impossible and are dropped, and
 * of the rest the one with the lower reprojection error is returned.
 *
 * @param marker_size side of the black square, metres.
 * @param K 3x3 camera matrix.
 * @param dist distortion coefficients (may be empty).
 * @return the pose, or nothing if no physically possible solution exists.
 */
std::optional<PoseEstimate> estimate_marker_pose(
  const std::array<cv::Point2f, 4> & corners, double marker_size,
  const cv::Matx33d & K, const std::vector<double> & dist);

/**
 * Express a marker pose from the camera optical frame in the robot base frame.
 *
 * The controller works in base_link, where x is forward, so this turns a camera-frame pose
 * into the range and alignment the robot must act on.
 *
 * @param T_optical_marker marker pose in the optical frame.
 * @param T_base_optical optical frame pose in the base frame (from the robot description).
 */
Eigen::Isometry3d transform_pose(
  const Eigen::Isometry3d & T_optical_marker, const Eigen::Isometry3d & T_base_optical);

/**
 * Ground-plane range and bearing of a point in the base frame.
 *
 * These two numbers are what the docking controller steers on: how far to drive and which
 * way to turn.
 */
RangeBearing range_and_bearing(const Eigen::Vector3d & position);

/**
 * Draw marker outlines, IDs and pose axes on a colour image for the debug view.
 *
 * Lets a person check in RViz that the detector sees the right marker and that the normal
 * (z axis) points out of the face toward the robot, which is the docking direction.
 *
 * @param image 8-bit three-channel image, modified in place.
 * @param marker_size side of the black square, metres (sets the axis length).
 * @throws std::invalid_argument if the image is not 8-bit three-channel.
 */
void draw_debug(
  cv::Mat & image, const std::vector<MarkerObservation> & observations,
  const cv::Matx33d & K, const std::vector<double> & dist, double marker_size);

}  // namespace dockbot_perception

#endif  // DOCKBOT_PERCEPTION__POSE_ESTIMATION_HPP_
