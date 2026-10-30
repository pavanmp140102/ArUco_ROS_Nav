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
// @Description: Node tests PN1 to PN4 for the ArUco detector, run in one process with an executor.

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "dockbot_interfaces/msg/aruco_marker_array.hpp"
#include "dockbot_perception/aruco_detector.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "synthetic_marker.hpp"

using dockbot_perception::testing::dockbot_camera;
using dockbot_perception::testing::draw_synthetic_marker;
using dockbot_perception::testing::marker_pose;
using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;

/// Test fixture: a detector node plus a "camera" node that feeds it, all on one executor.
class DetectorNodeTest : public ::testing::Test
{
protected:
  /// Start rclcpp and load the camera model and dictionary the tests render markers with.
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    camera_ = dockbot_camera();
    dict_ = dockbot_perception::get_dictionary("DICT_4X4_50");
  }

  /// Release the executor and shut rclcpp down so the next test starts clean.
  void TearDown() override
  {
    executor_.reset();
    rclcpp::shutdown();
  }

  /// Create the detector under test, the feeding node and the listener for its output.
  void start(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    detector_ = std::make_shared<dockbot_perception::ArucoDetector>(options);
    feeder_ = std::make_shared<rclcpp::Node>("feeder");
    image_pub_ = feeder_->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS());
    info_pub_ = feeder_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", rclcpp::SensorDataQoS());
    markers_sub_ = feeder_->create_subscription<dockbot_interfaces::msg::ArucoMarkerArray>(
      "/aruco/markers", 100,
      [this](dockbot_interfaces::msg::ArucoMarkerArray::ConstSharedPtr msg) {
        received_.push_back(*msg);
      });
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(feeder_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, feeder_, false);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(feeder_);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(detector_);
    executor_->add_node(feeder_);
  }

  /// Spin both nodes until the condition holds or the wall-clock time is up.
  template<typename Condition>
  bool spin_until(Condition done, std::chrono::milliseconds limit)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(10ms);
      if (done()) {
        return true;
      }
    }
    return done();
  }

  /// Publish the camera mount: base_link -> camera_optical_frame, 0.05 m behind the axle.
  void publish_mount()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = feeder_->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "camera_optical_frame";
    tf.transform.translation.x = -0.05;
    tf.transform.translation.z = 0.20;
    const Eigen::Quaterniond q(
      (Eigen::AngleAxisd(-kPi / 2, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-kPi / 2, Eigen::Vector3d::UnitX())).toRotationMatrix());
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    static_broadcaster_->sendTransform(tf);
  }

  /// Publish the camera intrinsics the detector needs before it can estimate marker poses.
  void publish_info()
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = "camera_optical_frame";
    info.width = 1280;
    info.height = 720;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        info.k[3 * r + c] = camera_.K(r, c);
      }
    }
    info.d = {0, 0, 0, 0, 0};
    info_pub_->publish(info);
  }

  /// Render a grey image with the given marker (or none) and wrap it as a ROS image.
  sensor_msgs::msg::Image make_image(
    const std::vector<std::pair<int, Eigen::Isometry3d>> & markers, rclcpp::Time stamp)
  {
    cv::Mat img(720, 1280, CV_8UC1, cv::Scalar(140));
    for (const auto & [id, T] : markers) {
      draw_synthetic_marker(img, camera_, dict_, 4, id, 0.15, T);
    }
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = "camera_optical_frame";
    return *cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img).toImageMsg();
  }

  dockbot_perception::testing::SyntheticCamera camera_;
  dockbot_perception::Dictionary dict_;
  std::shared_ptr<dockbot_perception::ArucoDetector> detector_;
  std::shared_ptr<rclcpp::Node> feeder_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<dockbot_interfaces::msg::ArucoMarkerArray>::SharedPtr markers_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<dockbot_interfaces::msg::ArucoMarkerArray> received_;
};

}  // namespace

// PN1: image + intrinsics + mount in, one array in base_link out.
TEST_F(DetectorNodeTest, PN1_PublishesMarkerInBaseLink)
{
  start();
  publish_mount();
  publish_info();
  // Marker 1 m straight ahead of the camera, so 0.95 m ahead of base_link.
  const auto image = make_image({{2, marker_pose(0.0, 0.0, 1.0, 0.0)}}, feeder_->now());
  // Keep publishing until the detector has the intrinsics and the transform.
  ASSERT_TRUE(
    spin_until(
      [&]() {
        image_pub_->publish(image);
        return !received_.empty();
      }, 5000ms));

  const auto & out = received_.back();
  EXPECT_EQ(out.header.frame_id, "base_link");
  EXPECT_EQ(out.header.stamp, image.header.stamp);
  ASSERT_EQ(out.markers.size(), 1u);
  const auto & m = out.markers[0];
  EXPECT_EQ(m.id, 2);
  EXPECT_NEAR(m.pose.position.x, 0.95, 0.02);
  EXPECT_NEAR(m.pose.position.y, 0.0, 0.02);
  EXPECT_NEAR(m.range, 0.95, 0.02);
  EXPECT_NEAR(m.bearing, 0.0, 0.02);
  EXPECT_LT(m.reprojection_error, 2.0);
  // The marker's z axis (docking normal) points back at the robot: -x in base_link.
  const Eigen::Quaterniond q(
    m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z);
  EXPECT_NEAR((q * Eigen::Vector3d::UnitZ()).x(), -1.0, 0.02);
}

// PN2: images before the intrinsics produce no output, a warning, and no crash.
TEST_F(DetectorNodeTest, PN2_ImagesBeforeCameraInfo)
{
  start();
  publish_mount();
  const auto image = make_image({{2, marker_pose(0.0, 0.0, 1.0, 0.0)}}, feeder_->now());
  spin_until(
    [&]() {
      image_pub_->publish(image);
      return false;
    }, 1000ms);
  EXPECT_TRUE(received_.empty());

  // Once the intrinsics arrive the node starts producing output.
  publish_info();
  EXPECT_TRUE(
    spin_until(
      [&]() {
        image_pub_->publish(image);
        return !received_.empty();
      }, 5000ms));
}

// PN3: sustained 30 Hz of full-size images: one message per image, empty ones included.
TEST_F(DetectorNodeTest, PN3_RateAndEmptyArrays)
{
  start({rclcpp::Parameter("publish_debug_image", false)});
  publish_mount();
  publish_info();
  const auto with_marker = make_image({{2, marker_pose(0.0, 0.0, 1.5, 0.3)}}, feeder_->now());
  const auto empty = make_image({}, feeder_->now());

  // Wait until the detector is ready, then count from a clean start.
  ASSERT_TRUE(
    spin_until(
      [&]() {
        image_pub_->publish(empty);
        return !received_.empty();
      }, 5000ms));
  spin_until([]() {return false;}, 500ms);  // Let warm-up images still in flight drain.
  received_.clear();

  int published = 0;
  const auto start_time = std::chrono::steady_clock::now();
  auto timer = feeder_->create_wall_timer(
    std::chrono::microseconds(33333), [&]() {
      auto msg = (published % 3 == 2) ? empty : with_marker;
      msg.header.stamp = feeder_->now();
      image_pub_->publish(msg);
      ++published;
    });
  spin_until([]() {return false;}, 10000ms);
  timer->cancel();
  spin_until([]() {return false;}, 300ms);  // Let the last images drain.
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

  const double rate = static_cast<double>(received_.size()) / elapsed;
  EXPECT_GE(rate, 20.0);
  EXPECT_EQ(static_cast<int>(received_.size()), published);
  int empties = 0;
  for (const auto & msg : received_) {
    empties += msg.markers.empty() ? 1 : 0;
  }
  EXPECT_GT(empties, published / 4);
  EXPECT_LT(empties, published / 2);
}

// PN4: with publish_tf on, base_link -> aruco_<id> matches the message pose.
TEST_F(DetectorNodeTest, PN4_PublishesTf)
{
  start({rclcpp::Parameter("publish_tf", true)});
  publish_mount();
  publish_info();
  const auto image = make_image({{4, marker_pose(0.2, 0.0, 1.2, 0.2)}}, feeder_->now());
  ASSERT_TRUE(
    spin_until(
      [&]() {
        image_pub_->publish(image);
        return !received_.empty() && tf_buffer_->canTransform(
          "base_link", "aruco_4", tf2::TimePointZero);
      }, 5000ms));

  const auto tf = tf_buffer_->lookupTransform("base_link", "aruco_4", tf2::TimePointZero);
  const auto & m = received_.back().markers[0];
  EXPECT_NEAR(tf.transform.translation.x, m.pose.position.x, 1e-6);
  EXPECT_NEAR(tf.transform.translation.y, m.pose.position.y, 1e-6);
  EXPECT_NEAR(tf.transform.translation.z, m.pose.position.z, 1e-6);
  EXPECT_NEAR(tf.transform.rotation.w, m.pose.orientation.w, 1e-6);
}
