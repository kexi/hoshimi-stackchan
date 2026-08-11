#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace connectivity {

struct GeoIpLocation {
  double latitudeDegrees = 0.0;
  double longitudeEastDegrees = 0.0;
  bool valid = false;
};

[[nodiscard]] bool isValidLocation(double latitudeDegrees, double longitudeEastDegrees);

// ipwho.isの必要最小限のJSON応答から観測地点を取り出す。
[[nodiscard]] GeoIpLocation parseGeoIpResponse(std::string_view response);

// Wi-Fi STA MACの末尾3バイトから stackchan-xxxxxx を作る。
[[nodiscard]] std::array<char, 17> stackchanHostName(const std::array<std::uint8_t, 6>& macAddress);

} // namespace connectivity
