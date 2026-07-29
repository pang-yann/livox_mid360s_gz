// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2026 pang-yann
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

#include <gz/msgs/pointcloud_packed.pb.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "livox_mid360s_gz/point_cloud_sampler.hpp"
#include "livox_mid360s_gz/scan_pattern.hpp"

namespace livox_mid360s_gz
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

template<typename T>
T SdfValue(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & key,
  const T & fallback)
{
  return sdf->Get<T>(key, fallback).first;
}

}  // namespace

class LivoxMid360sSystem final
  : public gz::sim::System,
  public gz::sim::ISystemConfigure
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    const std::string input_topic = SdfValue<std::string>(sdf, "input_topic", "");
    const std::string output_topic = SdfValue<std::string>(
      sdf, "output_topic", input_topic + "/livox");
    if (input_topic.empty() || output_topic.empty()) {
      gzerr << "LivoxMid360sSystem requires input_topic and output_topic.\n";
      return;
    }

    std::filesystem::path pattern_path = SdfValue<std::string>(sdf, "scan_pattern", "");
    if (pattern_path.empty()) {
      pattern_path =
        ament_index_cpp::get_package_share_directory(
        "livox_mid360s_gz") /
        std::filesystem::path("scan_patterns/mid360.csv");
    } else if (pattern_path.is_relative()) {
      pattern_path =
        ament_index_cpp::get_package_share_directory(
        "livox_mid360s_gz") /
        std::filesystem::path("scan_patterns") / pattern_path;
    }

    const double azimuth_offset = SdfValue<double>(
      sdf, "azimuth_offset", -kPi);
    const double elevation_offset = SdfValue<double>(sdf, "elevation_offset", 0.0);
    const auto line_count = SdfValue<unsigned int>(sdf, "line_count", 4);

    std::string error;
    if (!pattern_.Load(
        pattern_path, azimuth_offset, elevation_offset, line_count, error))
    {
      gzerr << "LivoxMid360sSystem: " << error << "\n";
      return;
    }

    SamplerConfig config;
    config.horizontal_min = SdfValue<double>(
      sdf, "horizontal_min", -kPi);
    config.horizontal_max = SdfValue<double>(
      sdf, "horizontal_max", kPi);
    config.vertical_min = SdfValue<double>(
      sdf, "vertical_min", -7.22 * kPi / 180.0);
    config.vertical_max = SdfValue<double>(
      sdf, "vertical_max", 55.22 * kPi / 180.0);
    config.point_rate = SdfValue<double>(sdf, "point_rate", 200000.0);
    const double scan_rate = SdfValue<double>(sdf, "scan_rate", 10.0);
    const std::size_t default_points = scan_rate > 0.0 ?
      static_cast<std::size_t>(std::llround(config.point_rate / scan_rate)) : 0;
    config.points_per_frame = SdfValue<unsigned int>(
      sdf, "points_per_frame", static_cast<unsigned int>(default_points));
    config.drop_invalid_points = SdfValue<bool>(sdf, "drop_invalid_points", true);
    sampler_ = std::make_unique<PointCloudSampler>(config);

    publisher_ = node_.Advertise<gz::msgs::PointCloudPacked>(output_topic);
    if (!publisher_) {
      gzerr << "LivoxMid360sSystem cannot advertise [" << output_topic << "].\n";
      sampler_.reset();
      return;
    }
    if (!node_.Subscribe(input_topic, &LivoxMid360sSystem::OnPointCloud, this)) {
      gzerr << "LivoxMid360sSystem cannot subscribe [" << input_topic << "].\n";
      sampler_.reset();
      return;
    }

    gzmsg << "LivoxMid360sSystem sampling [" << input_topic << "] to ["
          << output_topic << "] with " << pattern_.Size() << " directions.\n";
  }

private:
  void OnPointCloud(const gz::msgs::PointCloudPacked & input)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sampler_) {
      return;
    }

    gz::msgs::PointCloudPacked output;
    std::string error;
    if (!sampler_->Sample(input, pattern_, cursor_, output, error)) {
      if (!reported_sampling_error_) {
        gzerr << "LivoxMid360sSystem: " << error << "\n";
        reported_sampling_error_ = true;
      }
      return;
    }
    reported_sampling_error_ = false;
    publisher_.Publish(output);
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  ScanPattern pattern_;
  std::unique_ptr<PointCloudSampler> sampler_;
  std::size_t cursor_{0};
  std::mutex mutex_;
  bool reported_sampling_error_{false};
};

}  // namespace livox_mid360s_gz

GZ_ADD_PLUGIN(
  livox_mid360s_gz::LivoxMid360sSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(
  livox_mid360s_gz::LivoxMid360sSystem,
  "livox_mid360s_gz::LivoxMid360sSystem")
