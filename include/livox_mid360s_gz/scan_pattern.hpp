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

#ifndef LIVOX_MID360S_GZ__SCAN_PATTERN_HPP_
#define LIVOX_MID360S_GZ__SCAN_PATTERN_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace livox_mid360s_gz
{

struct ScanDirection
{
  double azimuth{};
  double elevation{};
  std::uint8_t line{};
};

class ScanPattern
{
public:
  bool Load(
    const std::filesystem::path & path,
    double azimuth_offset,
    double elevation_offset,
    std::size_t line_count,
    std::string & error);

  const ScanDirection & At(std::size_t index) const;
  std::size_t Size() const;
  bool Empty() const;

private:
  std::vector<ScanDirection> directions_;
};

}  // namespace livox_mid360s_gz

#endif  // LIVOX_MID360S_GZ__SCAN_PATTERN_HPP_
