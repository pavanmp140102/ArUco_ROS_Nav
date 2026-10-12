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

#include "dockbot_perception/aruco_detector.hpp"

#include <cmath>
#include <utility>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace dockbot_perception
{

namespace
{

/// Declare a string parameter with a description, so `ros2 param describe` documents it.
std::string declare_string(
  rclcpp::Node & node, const std::string & name, const std::string & value,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  return node.declare_parameter<std::string>(name, value, d);
}

/// Declare a bool parameter with a description.
bool declare_bool(
  rclcpp::Node & node, const std::string & name, bool value, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  return node.declare_parameter<bool>(name, value, d);
}

/// Declare a positive double parameter with a description and an upper bound.
double declare_positive(
  rclcpp::Node & node, const std::string & name, double value, double max,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = 1e-6;
  range.to_value = max;
  d.floating_point_range.push_back(range);
  return node.declare_parameter<double>(name, value, d);
}

/// Build the rigid transform of a TF message, to combine it with the pose maths.
Eigen::Isometry3d to_isometry(const geometry_msgs::msg::Transform & t)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)
    .normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  return T;
}

}  // namespace

ArucoDetector::ArucoDetector(const rclcpp::NodeOptions & options)
: rclcpp::Node("aruco_detector", options)
{
  const auto image_topic = declare_string(*this, "image_topic", "/camera/image_raw", "Input image");
  const auto info_topic =
    declare_string(*this, "camera_info_topic", "/camera/camera_info", "Input camera intrinsics");
  const auto dictionary_name = declare_string(
    *this, "dictionary", "DICT_4X4_50", "OpenCV ArUco dictionary name");
  marker_size_ = declare_positive(
    *this, "marker_size", 0.15, 1.0, "Side of the black square, metres");
  base_frame_ = declare_string(
    *this, "base_frame", "base_link", "Frame the poses are published in");
  max_reprojection_error_ = declare_positive(
    *this, "max_reprojection_error", 2.0, 100.0,
      "Detections above this error (pixels) are dropped");
  publish_tf_ = declare_bool(
    *this, "publish_tf", true, "Broadcast base_frame -> <tf_prefix><id> for each marker");
  tf_prefix_ = declare_string(*this, "tf_prefix", "aruco_", "Prefix for marker frame names");
  publish_debug_ = declare_bool(
    *this, "publish_debug_image", true, "Publish /aruco/debug_image when someone subscribes");

  dictionary_ = get_dictionary(dictionary_name);  // Throws on an unknown name: fail at start-up.

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  markers_pub_ = create_publisher<dockbot_interfaces::msg::ArucoMarkerArray>(
    "aruco/markers", rclcpp::QoS(10));
  debug_pub_ = create_publisher<sensor_msgs::msg::Image>("aruco/debug_image", rclcpp::QoS(1));
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    info_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {on_camera_info(msg);});
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(msg);});
}

void ArucoDetector::on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (K_.has_value() || msg->k[0] <= 0.0) {
    return;  // Intrinsics are fixed for a simulated camera; an all-zero matrix means "unset".
  }
  K_ = cv::Matx33d(
    msg->k[0], msg->k[1], msg->k[2],
    msg->k[3], msg->k[4], msg->k[5],
    msg->k[6], msg->k[7], msg->k[8]);
  dist_.assign(msg->d.begin(), msg->d.end());
  RCLCPP_INFO(get_logger(), "Camera intrinsics received (fx = %.1f)", msg->k[0]);
}

bool ArucoDetector::ensure_base_from_optical(const std::string & optical_frame)
{
  if (T_base_optical_.has_value()) {
    return true;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(base_frame_, optical_frame, tf2::TimePointZero);
    T_base_optical_ = to_isometry(tf.transform);
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Waiting for transform %s -> %s: %s",
      base_frame_.c_str(), optical_frame.c_str(), e.what());
    return false;
  }
}

void ArucoDetector::on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!K_.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera intrinsics");
    return;
  }
  if (!ensure_base_from_optical(msg->header.frame_id)) {
    return;
  }

  cv_bridge::CvImageConstPtr gray;
  try {
    gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Cannot read image: %s", e.what());
    return;
  }

  dockbot_interfaces::msg::ArucoMarkerArray out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = base_frame_;
  std::vector<MarkerObservation> observations;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  for (const auto & detection : detect_markers(gray->image, dictionary_)) {
    const auto pose = estimate_marker_pose(detection.corners, marker_size_, *K_, dist_);
    if (!pose || pose->reprojection_error > max_reprojection_error_) {
      continue;
    }
    observations.push_back({detection, *pose});

    const Eigen::Isometry3d T_base_marker = transform_pose(pose->T_optical_marker,
        *T_base_optical_);
    const Eigen::Quaterniond q(T_base_marker.linear());
    const Eigen::Vector3d p = T_base_marker.translation();
    const RangeBearing rb = range_and_bearing(p);

    dockbot_interfaces::msg::ArucoMarker marker;
    marker.id = detection.id;
    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.range = static_cast<float>(rb.range);
    marker.bearing = static_cast<float>(rb.bearing);
    marker.reprojection_error = static_cast<float>(pose->reprojection_error);
    out.markers.push_back(marker);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = out.header;
    tf.child_frame_id = tf_prefix_ + std::to_string(detection.id);
    tf.transform.translation.x = p.x();
    tf.transform.translation.y = p.y();
    tf.transform.translation.z = p.z();
    tf.transform.rotation = marker.pose.orientation;
    transforms.push_back(tf);
  }

  markers_pub_->publish(out);
  if (publish_tf_ && !transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
  if (publish_debug_ && debug_pub_->get_subscription_count() > 0) {
    publish_debug_image(msg, observations);
  }
}

void ArucoDetector::publish_debug_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const std::vector<MarkerObservation> & observations)
{
  try {
    cv_bridge::CvImagePtr color =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    draw_debug(color->image, observations, *K_, dist_, marker_size_);
    debug_pub_->publish(*color->toImageMsg());
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Debug image failed: %s", e.what());
  }
}

}  // namespace dockbot_perception

RCLCPP_COMPONENTS_REGISTER_NODE(dockbot_perception::ArucoDetector)
