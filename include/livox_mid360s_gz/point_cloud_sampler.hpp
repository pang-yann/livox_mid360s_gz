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

#ifndef LIVOX_MID360S_GZ__POINT_CLOUD_SAMPLER_HPP_
#define LIVOX_MID360S_GZ__POINT_CLOUD_SAMPLER_HPP_

#include <gz/msgs/pointcloud_packed.pb.h>

#include <cstddef>
#include <string>

#include "livox_mid360s_gz/scan_pattern.hpp"

namespace livox_mid360s_gz
{

struct SamplerConfig
{
  double horizontal_min{-3.14159265358979323846};
  double horizontal_max{3.14159265358979323846};
  double vertical_min{-0.126012280626355};
  double vertical_max{0.963770320680978};
  double point_rate{200000.0};
  std::size_t points_per_frame{20000};
  bool drop_invalid_points{true};
};

class PointCloudSampler
{
public:
  explicit PointCloudSampler(SamplerConfig config);

  bool Sample(
    const gz::msgs::PointCloudPacked & input,
    const ScanPattern & pattern,
    std::size_t & cursor,
    gz::msgs::PointCloudPacked & output,
    std::string & error) const;

private:
  SamplerConfig config_;
};

}  // namespace livox_mid360s_gz

#endif  // LIVOX_MID360S_GZ__POINT_CLOUD_SAMPLER_HPP_
