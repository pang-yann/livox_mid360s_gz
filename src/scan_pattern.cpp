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

#include "livox_mid360s_gz/scan_pattern.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace livox_mid360s_gz
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double NormalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

std::string CanonicalCsvToken(const std::string & token)
{
  std::string cleaned = token;
  cleaned.erase(
    cleaned.begin(),
    std::find_if(
      cleaned.begin(),
      cleaned.end(),
      [](const unsigned char ch) {return !std::isspace(ch);}));
  cleaned.erase(
    std::find_if(
      cleaned.rbegin(),
      cleaned.rend(),
      [](const unsigned char ch) {return !std::isspace(ch);}).base(),
    cleaned.end());
  if (cleaned.size() >= 3 &&
    static_cast<unsigned char>(cleaned[0]) == 0xEF &&
    static_cast<unsigned char>(cleaned[1]) == 0xBB &&
    static_cast<unsigned char>(cleaned[2]) == 0xBF)
  {
    cleaned.erase(0, 3);
  }
  std::transform(
    cleaned.begin(),
    cleaned.end(),
    cleaned.begin(),
    [](const unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  return cleaned;
}

bool IsKnownHeader(
  const std::string & time_text,
  const std::string & azimuth_text,
  const std::string & zenith_text)
{
  return CanonicalCsvToken(time_text) == "time/s" &&
         CanonicalCsvToken(azimuth_text) == "azimuth/deg" &&
         CanonicalCsvToken(zenith_text) == "zenith/deg";
}

}  // namespace

bool ScanPattern::Load(
  const std::filesystem::path & path,
  const double azimuth_offset,
  const double elevation_offset,
  const std::size_t line_count,
  std::string & error)
{
  if (line_count == 0 || line_count > 256) {
    error = "line_count must be in [1, 256]";
    return false;
  }

  std::ifstream stream(path);
  if (!stream.is_open()) {
    error = "cannot open scan pattern: " + path.string();
    return false;
  }

  std::vector<ScanDirection> parsed;
  std::string line;
  std::size_t source_line = 0;
  while (std::getline(stream, line)) {
    ++source_line;
    if (line.empty()) {
      continue;
    }

    std::stringstream row(line);
    std::string time_text;
    std::string azimuth_text;
    std::string zenith_text;
    if (!std::getline(row, time_text, ',') ||
      !std::getline(row, azimuth_text, ',') ||
      !std::getline(row, zenith_text, ','))
    {
      error = "invalid CSV row " + std::to_string(source_line);
      return false;
    }

    if (source_line == 1 && IsKnownHeader(time_text, azimuth_text, zenith_text)) {
      continue;
    }

    try {
      std::size_t azimuth_end = 0;
      std::size_t zenith_end = 0;
      const double azimuth_degrees = std::stod(azimuth_text, &azimuth_end);
      const double zenith_degrees = std::stod(zenith_text, &zenith_end);
      if (azimuth_end != azimuth_text.size() || zenith_end != zenith_text.size()) {
        throw std::invalid_argument("trailing characters");
      }

      ScanDirection direction;
      direction.azimuth = NormalizeAngle(
        azimuth_degrees * kPi / 180.0 + azimuth_offset);
      direction.elevation =
        kPi / 2.0 - zenith_degrees * kPi / 180.0 +
        elevation_offset;
      direction.line = static_cast<std::uint8_t>(parsed.size() % line_count);
      parsed.push_back(direction);
    } catch (const std::exception &) {
      error = "invalid numeric value in CSV row " + std::to_string(source_line);
      return false;
    }
  }

  if (parsed.empty()) {
    error = "scan pattern has no points: " + path.string();
    return false;
  }

  directions_ = std::move(parsed);
  error.clear();
  return true;
}

const ScanDirection & ScanPattern::At(const std::size_t index) const
{
  return directions_.at(index % directions_.size());
}

std::size_t ScanPattern::Size() const
{
  return directions_.size();
}

bool ScanPattern::Empty() const
{
  return directions_.empty();
}

}  // namespace livox_mid360s_gz
