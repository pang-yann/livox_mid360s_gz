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

#include "livox_mid360s_gz/point_cloud_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>

#include <gz/msgs/Utility.hh>

namespace livox_mid360s_gz
{
namespace
{

using Field = gz::msgs::PointCloudPacked::Field;
constexpr double kPi = 3.14159265358979323846;

std::optional<std::uint32_t> FloatFieldOffset(
  const gz::msgs::PointCloudPacked & cloud,
  const std::string & name)
{
  for (const auto & field : cloud.field()) {
    if (field.name() == name && field.datatype() == Field::FLOAT32 && field.count() == 1) {
      return field.offset();
    }
  }
  return std::nullopt;
}

template<typename T>
T Read(const std::string & data, const std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

template<typename T>
void Write(
  std::string & data,
  const std::size_t point_offset,
  const std::uint32_t field_offset,
  const T value)
{
  std::memcpy(data.data() + point_offset + field_offset, &value, sizeof(T));
}

std::uint32_t FieldOffset(
  const gz::msgs::PointCloudPacked & cloud,
  const std::string & name)
{
  for (const auto & field : cloud.field()) {
    if (field.name() == name) {
      return field.offset();
    }
  }
  return 0;
}

double WrapIntoRange(double angle, const double minimum, const double maximum)
{
  const double span = maximum - minimum;
  if (span >= 2.0 * kPi - 1e-6) {
    while (angle < minimum) {
      angle += 2.0 * kPi;
    }
    while (angle > maximum) {
      angle -= 2.0 * kPi;
    }
  }
  return angle;
}

}  // namespace

PointCloudSampler::PointCloudSampler(SamplerConfig config)
: config_(std::move(config))
{
}

bool PointCloudSampler::Sample(
  const gz::msgs::PointCloudPacked & input,
  const ScanPattern & pattern,
  std::size_t & cursor,
  gz::msgs::PointCloudPacked & output,
  std::string & error) const
{
  if (pattern.Empty()) {
    error = "scan pattern is empty";
    return false;
  }
  if (input.width() < 1 || input.height() < 1 || input.point_step() < 1) {
    error = "input point cloud has invalid dimensions";
    return false;
  }
  const std::size_t min_row_step =
    static_cast<std::size_t>(input.width()) * input.point_step();
  if (input.row_step() < min_row_step) {
    error = "input point cloud has inconsistent row_step";
    return false;
  }
  if (input.is_bigendian()) {
    error = "big-endian point clouds are unsupported";
    return false;
  }
  if (config_.horizontal_max <= config_.horizontal_min ||
    config_.vertical_max <= config_.vertical_min ||
    config_.point_rate <= 0.0 ||
    config_.points_per_frame == 0)
  {
    error = "sampler configuration is invalid";
    return false;
  }

  const auto x_offset = FloatFieldOffset(input, "x");
  const auto y_offset = FloatFieldOffset(input, "y");
  const auto z_offset = FloatFieldOffset(input, "z");
  const auto intensity_offset = FloatFieldOffset(input, "intensity");
  if (!x_offset || !y_offset || !z_offset || !intensity_offset) {
    error = "input requires FLOAT32 x, y, z, and intensity fields";
    return false;
  }
  if (*x_offset + sizeof(float) > input.point_step() ||
    *y_offset + sizeof(float) > input.point_step() ||
    *z_offset + sizeof(float) > input.point_step() ||
    *intensity_offset + sizeof(float) > input.point_step())
  {
    error = "input point cloud field offsets exceed point_step";
    return false;
  }

  const std::size_t expected_size =
    static_cast<std::size_t>(input.row_step()) * input.height();
  if (input.data().size() < expected_size) {
    error = "input point cloud data is truncated";
    return false;
  }

  output.Clear();
  gz::msgs::InitPointCloudPacked(
    output, "", false,
    {
      {"xyz", Field::FLOAT32},
      {"intensity", Field::FLOAT32},
      {"ring", Field::UINT16},
      {"time", Field::FLOAT32},
      {"offset_time", Field::UINT32},
      {"line", Field::UINT8},
      {"tag", Field::UINT8},
    });
  if (input.has_header()) {
    output.mutable_header()->CopyFrom(input.header());
  }
  output.set_height(1);
  output.set_is_bigendian(false);

  const auto output_x = FieldOffset(output, "x");
  const auto output_y = FieldOffset(output, "y");
  const auto output_z = FieldOffset(output, "z");
  const auto output_intensity = FieldOffset(output, "intensity");
  const auto output_ring = FieldOffset(output, "ring");
  const auto output_time = FieldOffset(output, "time");
  const auto output_offset_time = FieldOffset(output, "offset_time");
  const auto output_line = FieldOffset(output, "line");
  const auto output_tag = FieldOffset(output, "tag");

  std::string & output_data = *output.mutable_data();
  output_data.resize(config_.points_per_frame * output.point_step());
  std::size_t output_count = 0;
  bool has_invalid = false;

  const double horizontal_span = config_.horizontal_max - config_.horizontal_min;
  const double vertical_span = config_.vertical_max - config_.vertical_min;

  for (std::size_t i = 0; i < config_.points_per_frame; ++i) {
    const ScanDirection & direction = pattern.At(cursor + i);
    const double azimuth = WrapIntoRange(
      direction.azimuth, config_.horizontal_min, config_.horizontal_max);
    if (azimuth < config_.horizontal_min || azimuth > config_.horizontal_max ||
      direction.elevation < config_.vertical_min ||
      direction.elevation > config_.vertical_max)
    {
      continue;
    }

    const double horizontal_ratio =
      (azimuth - config_.horizontal_min) / horizontal_span;
    const double vertical_ratio =
      (direction.elevation - config_.vertical_min) / vertical_span;
    const auto column = static_cast<std::uint32_t>(std::llround(
        horizontal_ratio * static_cast<double>(input.width() - 1)));
    const auto row = static_cast<std::uint32_t>(std::llround(
        vertical_ratio * static_cast<double>(input.height() - 1)));
    const std::size_t input_point =
      static_cast<std::size_t>(row) * input.row_step() +
      static_cast<std::size_t>(column) * input.point_step();
    if (input_point + input.point_step() > input.data().size()) {
      error = "input point cloud layout points outside data buffer";
      return false;
    }

    const float source_x = Read<float>(input.data(), input_point + *x_offset);
    const float source_y = Read<float>(input.data(), input_point + *y_offset);
    const float source_z = Read<float>(input.data(), input_point + *z_offset);
    const float intensity = Read<float>(input.data(), input_point + *intensity_offset);
    const double range = std::hypot(
      static_cast<double>(source_x),
      static_cast<double>(source_y),
      static_cast<double>(source_z));
    const bool valid = std::isfinite(range) && range > 0.0;
    has_invalid = has_invalid || !valid;
    if (!valid && config_.drop_invalid_points) {
      continue;
    }

    const float output_range = valid ?
      static_cast<float>(range) : std::numeric_limits<float>::quiet_NaN();
    const float x = output_range * static_cast<float>(
      std::cos(direction.elevation) * std::cos(direction.azimuth));
    const float y = output_range * static_cast<float>(
      std::cos(direction.elevation) * std::sin(direction.azimuth));
    const float z = output_range * static_cast<float>(std::sin(direction.elevation));
    const float time = static_cast<float>(static_cast<double>(i) / config_.point_rate);
    const auto offset_time = static_cast<std::uint32_t>(std::llround(
        static_cast<double>(i) * 1e9 / config_.point_rate));
    const std::uint16_t ring = direction.line;
    const std::uint8_t tag = 0;

    const std::size_t output_point = output_count * output.point_step();
    Write(output_data, output_point, output_x, x);
    Write(output_data, output_point, output_y, y);
    Write(output_data, output_point, output_z, z);
    Write(output_data, output_point, output_intensity, intensity);
    Write(output_data, output_point, output_ring, ring);
    Write(output_data, output_point, output_time, time);
    Write(output_data, output_point, output_offset_time, offset_time);
    Write(output_data, output_point, output_line, direction.line);
    Write(output_data, output_point, output_tag, tag);
    ++output_count;
  }

  cursor = (cursor + config_.points_per_frame) % pattern.Size();
  output.set_width(output_count);
  output.set_row_step(output.point_step() * output.width());
  output_data.resize(output.row_step());
  output.set_is_dense(!has_invalid || config_.drop_invalid_points);
  error.clear();
  return true;
}

}  // namespace livox_mid360s_gz
