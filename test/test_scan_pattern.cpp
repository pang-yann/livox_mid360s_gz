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

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include "livox_mid360s_gz/scan_pattern.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(ScanPattern, LoadsAndTransformsOfficialColumns)
{
  livox_mid360s_gz::ScanPattern pattern;
  std::string error;
  ASSERT_TRUE(pattern.Load(
      std::filesystem::path(TEST_DATA_DIR) / "pattern.csv",
      -kPi, 0.0, 4, error)) << error;

  ASSERT_EQ(pattern.Size(), 4u);
  EXPECT_NEAR(pattern.At(0).azimuth, 0.0, 1e-12);
  EXPECT_NEAR(pattern.At(0).elevation, 0.0, 1e-12);
  EXPECT_NEAR(pattern.At(2).elevation, kPi / 4.0, 1e-12);
  EXPECT_EQ(pattern.At(3).line, 3u);
  EXPECT_NEAR(pattern.At(4).azimuth, pattern.At(0).azimuth, 1e-12);
}

TEST(ScanPattern, RejectsMissingFile)
{
  livox_mid360s_gz::ScanPattern pattern;
  std::string error;
  EXPECT_FALSE(pattern.Load("/does/not/exist.csv", 0.0, 0.0, 4, error));
  EXPECT_FALSE(error.empty());
}

TEST(ScanPattern, RejectsInvalidFirstRowThatIsNotHeader)
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() / "livox_mid360_bad_first_row.csv";

  {
    std::ofstream stream(path);
    ASSERT_TRUE(stream.is_open());
    stream << "not_a_header,not_a_number,90\n";
    stream << "1,270,90\n";
  }

  livox_mid360s_gz::ScanPattern pattern;
  std::string error;
  EXPECT_FALSE(pattern.Load(path, 0.0, 0.0, 4, error));
  EXPECT_FALSE(error.empty());

  std::error_code ec;
  std::filesystem::remove(path, ec);
}

}  // namespace
