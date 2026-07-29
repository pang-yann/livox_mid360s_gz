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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>

#include <gz/msgs/Utility.hh>

#include "livox_mid360s_gz/point_cloud_sampler.hpp"
#include "livox_mid360s_gz/scan_pattern.hpp"

namespace
{

using Field = gz::msgs::PointCloudPacked::Field;
constexpr double kPi = 3.14159265358979323846;

std::uint32_t Offset(
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

template<typename T>
void Write(
  gz::msgs::PointCloudPacked & cloud,
  const std::size_t index,
  const std::string & field,
  const T value)
{
  const std::size_t offset = index * cloud.point_step() + Offset(cloud, field);
  std::memcpy(cloud.mutable_data()->data() + offset, &value, sizeof(T));
}

template<typename T>
T Read(
  const gz::msgs::PointCloudPacked & cloud,
  const std::size_t index,
  const std::string & field)
{
  T value{};
  const std::size_t offset = index * cloud.point_step() + Offset(cloud, field);
  std::memcpy(&value, cloud.data().data() + offset, sizeof(T));
  return value;
}

TEST(PointCloudSampler, SelectsGridCellAndUsesExactPatternDirection)
{
  gz::msgs::PointCloudPacked input;
  gz::msgs::InitPointCloudPacked(
    input, "sensor", false,
    {{"xyz", Field::FLOAT32}, {"intensity", Field::FLOAT32}});
  input.set_width(3);
  input.set_height(1);
  input.set_row_step(input.width() * input.point_step());
  input.mutable_data()->resize(input.row_step());

  for (std::size_t i = 0; i < 3; ++i) {
    Write(input, i, "x", static_cast<float>(i + 1));
    Write(input, i, "y", 0.0F);
    Write(input, i, "z", 0.0F);
    Write(input, i, "intensity", static_cast<float>(10 + i));
  }

  livox_mid360s_gz::ScanPattern pattern;
  std::string error;
  ASSERT_TRUE(pattern.Load(
      std::filesystem::path(TEST_DATA_DIR) / "pattern.csv",
      -kPi, 0.0, 4, error)) << error;

  livox_mid360s_gz::SamplerConfig config;
  config.horizontal_min = -kPi;
  config.horizontal_max = kPi;
  config.vertical_min = -0.1;
  config.vertical_max = 0.1;
  config.points_per_frame = 2;
  config.point_rate = 200000.0;
  livox_mid360s_gz::PointCloudSampler sampler(config);

  std::size_t cursor = 0;
  gz::msgs::PointCloudPacked output;
  ASSERT_TRUE(sampler.Sample(input, pattern, cursor, output, error)) << error;

  ASSERT_EQ(output.width(), 2u);
  EXPECT_NEAR(Read<float>(output, 0, "x"), 2.0F, 1e-6);
  EXPECT_NEAR(Read<float>(output, 0, "y"), 0.0F, 1e-6);
  EXPECT_NEAR(Read<float>(output, 1, "x"), 0.0F, 1e-6);
  EXPECT_NEAR(Read<float>(output, 1, "y"), 3.0F, 1e-6);
  EXPECT_FLOAT_EQ(Read<float>(output, 1, "intensity"), 12.0F);
  EXPECT_EQ(Read<std::uint32_t>(output, 1, "offset_time"), 5000u);
  EXPECT_EQ(cursor, 2u);
  EXPECT_EQ(output.header().data(0).value(0), "sensor");
}

}  // namespace
