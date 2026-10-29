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
// @Description: ROS 2 node that turns camera images into marker poses relative to the robot.

#ifndef DOCKBOT_PERCEPTION__ARUCO_DETECTOR_HPP_
#define DOCKBOT_PERCEPTION__ARUCO_DETECTOR_HPP_

#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "dockbot_interfaces/msg/aruco_marker_array.hpp"
#include "dockbot_perception/pose_estimation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace dockbot_perception
{

/**
 * Detects ArUco markers in the camera image and publishes their poses in the robot base frame.
 *
 * This is the only source of position information for docking: the controller never sees an
 * image, only the ranges and bearings published here. Every image yields exactly one
 * ArucoMarkerArray, empty when nothing is seen, so consumers can tell "no marker" from "stalled".
 */
class ArucoDetector : public rclcpp::Node
{
public:
  /// Declare the parameters, subscribe to the camera topics and create the publishers.
  explicit ArucoDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// Remember the camera intrinsics (once), which the pose maths needs to turn pixels into metres.
  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  /// Detect markers in one image and publish their poses; the heart of the perception chain.
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /// Look up (and cache) the fixed transform from the camera frame to the base frame.
  bool ensure_base_from_optical(const std::string & optical_frame);

  /// Publish an annotated copy of the image so the detections can be checked in RViz.
  void publish_debug_image(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const std::vector<MarkerObservation> & observations);

  std::string base_frame_;
  std::string tf_prefix_;
  double marker_size_;
  double max_reprojection_error_;
  bool publish_tf_;
  bool publish_debug_;
  Dictionary dictionary_;

  std::optional<cv::Matx33d> K_;
  std::vector<double> dist_;
  std::optional<Eigen::Isometry3d> T_base_optical_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<dockbot_interfaces::msg::ArucoMarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace dockbot_perception

#endif  // DOCKBOT_PERCEPTION__ARUCO_DETECTOR_HPP_
