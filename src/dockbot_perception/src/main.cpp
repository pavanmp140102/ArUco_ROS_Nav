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
// @Description: Standalone executable that runs the ArUco detector node.

#include <memory>

#include "dockbot_perception/aruco_detector.hpp"
#include "rclcpp/rclcpp.hpp"

/// Start the detector as an ordinary process, the way the docking launch file runs it.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dockbot_perception::ArucoDetector>());
  rclcpp::shutdown();
  return 0;
}
