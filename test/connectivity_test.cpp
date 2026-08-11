#include "connectivity/geoip.hpp"
#include "test_support.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace {

void testHostNameUsesMacSuffix() {
  // MAC末尾3バイトが小文字16進数で一意なmDNS名になることを保証する。
  const std::array<std::uint8_t, 6> mac = {0x44, 0x1B, 0xF6, 0xDF, 0x59, 0x68};
  const auto hostname = connectivity::stackchanHostName(mac);
  CHECK_TRUE(std::string(hostname.data()) == "stackchan-df5968");
}

void testValidGeoIpResponse() {
  // 順序や空白に依存せずGeoIPの緯度経度を取り出せることを保証する。
  const connectivity::GeoIpLocation location = connectivity::parseGeoIpResponse(
      R"({"longitude": 140.106, "success" : true, "latitude":35.607})");
  CHECK_TRUE(location.valid);
  CHECK_NEAR(location.latitudeDegrees, 35.607, 1e-9);
  CHECK_NEAR(location.longitudeEastDegrees, 140.106, 1e-9);
}

void testInvalidGeoIpResponses() {
  // API失敗・欠落・型違い・範囲外の座標を観測地点として採用しないことを保証する。
  CHECK_TRUE(
      !connectivity::parseGeoIpResponse(R"({"success":false,"latitude":35.0,"longitude":140.0})")
           .valid);
  CHECK_TRUE(!connectivity::parseGeoIpResponse(R"({"success":true,"latitude":35.0})").valid);
  CHECK_TRUE(
      !connectivity::parseGeoIpResponse(R"({"success":true,"latitude":"35.0","longitude":140.0})")
           .valid);
  CHECK_TRUE(
      !connectivity::parseGeoIpResponse(R"({"success":true,"latitude":91.0,"longitude":140.0})")
           .valid);
  CHECK_TRUE(
      !connectivity::parseGeoIpResponse(R"({"success":true,"latitude":35.0oops,"longitude":140.0})")
           .valid);
  CHECK_TRUE(!connectivity::parseGeoIpResponse(R"({"success":t})").valid);
  CHECK_TRUE(!connectivity::parseGeoIpResponse(R"({"success":})").valid);
}

} // namespace

int main() {
  testHostNameUsesMacSuffix();
  testValidGeoIpResponse();
  testInvalidGeoIpResponses();
  return testing::summarize("connectivity");
}
