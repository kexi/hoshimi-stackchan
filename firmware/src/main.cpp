// compass-stackchan 本体。
//
// 真北と天体 (太陽・月・水星〜土星) の方向を、首で物理的に指し示す。
// yaw で方位を、pitch で高度を表す。
//
// 磁気測定前に首を正面へ戻す。首の角度によって方位が最大119度ずれ、横向きでは
// 磁場強度も変わるため、同じ測定姿勢の値だけを方位として採用する。
// ロジックは app_core 側にあり、ここは実機の入出力をstate machineへ橋渡しする。
//
// 状態はシリアルへ出す (just watch / just verify で読む)。
//
// Why not NVS: フラッシュの読み出しは esptool のリセットを伴うので、何度読んでも
// 「起動直後」の値しか取れず、指した結果が観測できない。NVS には節目の値だけを
// 30 秒間隔で残す。毎周期フラッシュへ書き、そこで球の当てはめまで回していた頃は、
// それ自体がウォッチドッグを踏んで数秒ごとに再起動していた。

#include <Avatar.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <M5StackChan.h>
#include <M5Unified.h>
#include <NetworkClientSecure.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_sntp.h>
#include <esp_system.h>

#include "app/presentation.hpp"
#include "app/state.hpp"
#include "compass/declination.hpp"
#include "compass/heading.hpp"
#include "compass/level_calibration.hpp"
#include "compass/stability.hpp"
#include "connectivity/geoip.hpp"
#include "location_config.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
// 実体は .gitignore 済み。無ければ Wi-Fi を使わず、真北だけを指すモードで動く。
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif

#ifndef NTP_SERVER_1
#define NTP_SERVER_1 "ntp.nict.jp"
#endif

#ifndef NTP_SERVER_2
#define NTP_SERVER_2 "pool.ntp.org"
#endif

#ifndef GEOIP_ENABLED
#define GEOIP_ENABLED true
#endif

#ifndef GEOIP_URL
#define GEOIP_URL "https://ipwho.is/?fields=success,latitude,longitude"
#endif

namespace {

constexpr const char* kLevelNamespace = "level";
constexpr const char* kGeoIpNamespace = "geoip";
// v5はサーボ停止直後の慣性を校正サンプルへ混ぜず、反転前の方位校正を破棄する。
constexpr std::uint32_t kLevelStorageVersion = 5;
constexpr std::uint32_t kGeoIpStorageVersion = 1;
constexpr int kMoveSpeed = 400;
constexpr std::uint32_t kGeoIpRefreshSeconds = 24 * 60 * 60;
constexpr std::uint32_t kNetworkRetryMillis = 5 * 60 * 1000;
constexpr std::uint16_t kGeoIpHttpTimeoutMillis = 5000;
constexpr int kControlHttpPort = 80;
constexpr int kTargetCount = static_cast<int>(astro::Target::kCount);

// 水平投影したキャリブレーションのサンプルを全点保持して円に当てはめる。
//
// Why not min/max: 各軸の端に到達した 2 点しか使わないので、端まで回しきらないと
// 中心が「回した範囲の中心」に寄る。実機ではそれで補正後の水平成分が地磁気の
// 6 割になり、方位が全方位に散らばった。最小二乗と被覆率を組み合わせ、
// 一周した水平円だけを採用する。
constexpr std::size_t kCalibrationCapacity = 512;
constexpr std::size_t kMinCalibrationSamples = 24;
constexpr float kMinFieldRadiusMicroTesla = 12.0F;
// 補正後に球へどれだけ乗っていれば採用するか。実機のハードアイアンは地磁気の
// 6 倍あり歪みが強いので、厳しくしすぎると永久に採用されない。
constexpr float kMaxCalibrationResidual = 0.25F;
// 当てはめは数百点の最小二乗で重い。毎周期回すとウォッチドッグを踏む。
constexpr std::uint32_t kFitIntervalMillis = 700;
// 静止中の同一点でバッファを埋めず、速度によらず一周分の異なる点だけを採る。
// BMM150の静止ノイズより十分広く、水平地磁気の円周からは24点以上採れる間隔。
constexpr float kDefaultCalibrationMinimumPointDistanceMicroTesla = 1.5F;
constexpr float kDefaultCalibrationMaxTiltDegrees = 10.0F;
constexpr float kDefaultCalibrationMinimumGravityDot = 0.9848078F;
constexpr float kCalibrationMinimumRotationRateDegreesPerSecond = 10.0F;
constexpr std::size_t kMagneticAverageWindow = 32;
// 状態をシリアルへ出す間隔。局面の遷移が追える程度に細かくする。
// NVS への書き込みはこれより間引く (フラッシュの摩耗を避けるため)。
constexpr std::uint32_t kProbeIntervalMillis = 500;
constexpr std::uint32_t kNvsProbeIntervalMillis = 30000;
constexpr int kServoArrivalToleranceDeci = compass::kMeasurementYawToleranceDeci;

struct ServoAngles {
  int yawDeci = 0;
  int pitchDeci = pointing::kPitchLevelDeci;
};

struct StoredGeoIpLocation {
  double latitudeDegrees = 0.0;
  double longitudeEastDegrees = 0.0;
  std::int64_t updatedUnixSeconds = 0;
};

enum class LocationSource : std::uint8_t {
  Fixed,
  CachedGeoIp,
  LiveGeoIp,
};

class BuiltInCertificateBundleClient : public NetworkClientSecure {
public:
  void useBuiltInCertificateBundle() {
    // Why not setInsecure(): GeoIPを改ざんされると、正しい天体でも別方向を指す。
    // ESP-IDF同梱の公開CA束を使い、特定サービスの中間証明書更新にも追従する。
    attach_ssl_certificate_bundle(sslclient.get(), true);
    _use_ca_bundle = true;
    _use_insecure = false;
  }
};

app::State g_state;
app::Config g_config;
astro::Observer g_observer;
std::array<char, 17> g_hostName{};
LocationSource g_locationSource = LocationSource::Fixed;
bool g_wifiConnected = false;
bool g_mdnsStarted = false;
bool g_mdnsAttempted = false;
bool g_ntpConfigured = false;
bool g_ntpSynchronized = false;
bool g_geoIpAttempted = false;
std::uint32_t g_lastMdnsAttemptMillis = 0;
std::uint32_t g_lastGeoIpAttemptMillis = 0;
std::atomic<bool> g_ntpSyncPending{false};
std::atomic<int> g_requestedTarget{-1};
StoredGeoIpLocation g_storedGeoIp;
bool g_hasStoredGeoIp = false;
WebServer g_controlServer{kControlHttpPort};
bool g_controlServerStarted = false;

struct CalibrationTuning {
  // 数値自体は小さいので内部RAMへ置く。大量の測定点だけをPSRAMへ逃がす。
  float maxTiltDegrees = kDefaultCalibrationMaxTiltDegrees;
  float minimumGravityDot = kDefaultCalibrationMinimumGravityDot;
  float minimumPointDistanceMicroTesla = kDefaultCalibrationMinimumPointDistanceMicroTesla;
};

// 試行値は再起動で安全な既定値へ戻す。合格した補正結果だけをNVSへ保存する。
CalibrationTuning g_calibrationTuning;

// 水平回転で取る補正。この機体は本体に強い磁石があり、傾けると磁石も一緒に
// 動くので 8 の字回しでは地磁気の球にならない (実機で半径 169uT = 地磁気の
// 3.7 倍になった)。首を固定して本体だけ水平に回せば相対関係が保たれる。
compass::LevelCalibration g_level;
// 直近の当てはめの被覆率。描画から読むだけにして、当てはめは間隔を空けて回す。
float g_calibrationCoverage = 0.0F;
float g_calibrationCandidateRadius = 0.0F;
float g_calibrationCandidateResidual = 1.0F;
std::uint32_t g_calibrationFitCount = 0;
std::uint32_t g_calibrationTiltResetCount = 0;
// キャリブレーション開始時の重力方向。ここから外れたサンプルは採らない。
compass::Vec3 g_referenceGravity;
// タッチ入力を受けた回数。実機でスワイプが検出されているかを見る。
int g_touchEventCount = 0;
// 3 ゾーンの強度を 1 つの整数に畳んだもの (前*100 + 中*10 + 後)。
// ジェスチャにならなくても、触れていること自体は分かる。
int g_touchIntensity = 0;
// 触れられていた周期の数。触っても 0 のままならセンサまで届いていない。
int g_touchSeenCount = 0;
// 直近の tick で サーボが静止していたか。診断用。
bool g_lastTickServoSettled = false;
compass::MeasurementGate g_gate;
// 平滑化を強めにする。
//
// 0.5 だと実機で方位が 12 度揺れ、それが首の指令にそのまま出て小刻みに
// 動き続けた。正面姿勢でも残る磁気ノイズを平均してから採用する。
compass::HeadingFilter g_headingFilter{0.15F};

compass::Vec3* g_calibrationSamples = nullptr;
std::size_t g_calibrationCount = 0;

bool g_timeValid = false;
bool g_headingValid = false;
float g_bodyTrueHeading = 0.0F;
esp_reset_reason_t g_bootResetReason = ESP_RST_UNKNOWN;
compass::Vec3 g_lastCoreMag;
compass::Vec3 g_lastRawMag;
compass::Vec3 g_lastSensorMag;
compass::Vec3 g_lastCoreAccel;
compass::Vec3 g_lastAccel;
compass::Vec3 g_lastObservedAccel;
compass::Vec3 g_lastCoreGyro;
compass::Vec3 g_lastFaceGyro;
float g_lastGyroMagnitude = 0.0F;
std::uint32_t g_lastGyroReadMillis = 0;
bool g_hasRawMagSample = false;
std::uint32_t g_lastMagChangeMillis = 0;
std::uint32_t g_magnetometerChangeCount = 0;
std::uint32_t g_magnetometerRepeatedCount = 0;
std::array<compass::Vec3, kMagneticAverageWindow> g_magneticAverageSamples{};
compass::Vec3 g_magneticAverageSum;
std::size_t g_magneticAverageIndex = 0;
std::size_t g_magneticAverageCount = 0;

std::uint32_t g_lastServoStopMillis = 0;
std::uint32_t g_lastObservedServoMotionMillis = 0;
int g_commandedYaw = 0;
int g_commandedPitch = pointing::kPitchLevelDeci;
ServoAngles g_lastServoAngles;

compass::Vec3 readMag() {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  M5.Imu.getMag(&x, &y, &z);
  g_lastCoreMag = compass::Vec3{x, y, z};
  return compass::stackChanFaceFrameFromCoreS3(g_lastCoreMag);
}

compass::Vec3 readAccel() {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  M5.Imu.getAccel(&x, &y, &z);
  g_lastCoreAccel = compass::Vec3{x, y, z};
  return compass::stackChanFaceFrameFromCoreS3(g_lastCoreAccel);
}

compass::Vec3 averageMagneticSample(compass::Vec3 sample) {
  const bool averageWindowIsFull = g_magneticAverageCount == kMagneticAverageWindow;
  if (averageWindowIsFull) {
    const compass::Vec3& oldest = g_magneticAverageSamples[g_magneticAverageIndex];
    g_magneticAverageSum.x -= oldest.x;
    g_magneticAverageSum.y -= oldest.y;
    g_magneticAverageSum.z -= oldest.z;
  } else {
    ++g_magneticAverageCount;
  }

  g_magneticAverageSamples[g_magneticAverageIndex] = sample;
  g_magneticAverageIndex = (g_magneticAverageIndex + 1) % kMagneticAverageWindow;
  g_magneticAverageSum.x += sample.x;
  g_magneticAverageSum.y += sample.y;
  g_magneticAverageSum.z += sample.z;

  const float sampleCount = static_cast<float>(g_magneticAverageCount);
  return {g_magneticAverageSum.x / sampleCount, g_magneticAverageSum.y / sampleCount,
          g_magneticAverageSum.z / sampleCount};
}

compass::Vec3 correctedHorizontalField(compass::Vec3 magneticField, compass::Vec3 acceleration) {
  const compass::Attitude attitude = compass::attitudeFromAccel(acceleration);
  const compass::Vec3 horizontal = compass::horizontalMagneticComponents(magneticField, attitude);
  return compass::applyLevelCalibration(g_level, horizontal);
}

float readGyroMagnitude() {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  M5.Imu.getGyro(&x, &y, &z);
  g_lastCoreGyro = compass::Vec3{x, y, z};
  g_lastFaceGyro = compass::stackChanFaceFrameFromCoreS3(g_lastCoreGyro);
  g_lastGyroMagnitude = std::sqrt(x * x + y * y + z * z);
  g_lastGyroReadMillis = millis();
  return g_lastGyroMagnitude;
}

bool setClockFromUnix(std::time_t unixSeconds) {
  const auto unix64 = static_cast<std::int64_t>(unixSeconds);
  const bool isPlausible = unix64 >= 1700000000LL && unix64 < 4102444800LL;
  if (!isPlausible) {
    return false;
  }

  const std::tm* utc = std::gmtime(&unixSeconds);
  if (utc == nullptr) {
    return false;
  }

  // OS時計とRTCを同じUTC秒から更新し、今の計算と次回起動の両方へ反映する。
  timeval systemClock = {unixSeconds, 0};
  settimeofday(&systemClock, nullptr);

  m5::rtc_datetime_t rtc;
  rtc.date.year = static_cast<std::uint16_t>(utc->tm_year + 1900);
  rtc.date.month = static_cast<std::uint8_t>(utc->tm_mon + 1);
  rtc.date.date = static_cast<std::uint8_t>(utc->tm_mday);
  rtc.time.hours = static_cast<std::uint8_t>(utc->tm_hour);
  rtc.time.minutes = static_cast<std::uint8_t>(utc->tm_min);
  rtc.time.seconds = static_cast<std::uint8_t>(utc->tm_sec);
  M5.Rtc.setDateTime(rtc);
  g_timeValid = true;
  return true;
}

bool isLevelCalibrationAcceptable(const compass::LevelCalibration& calibration) {
  const bool valuesAreFinite =
      std::isfinite(calibration.offsetX) && std::isfinite(calibration.offsetY) &&
      std::isfinite(calibration.scaleX) && std::isfinite(calibration.scaleY) &&
      std::isfinite(calibration.crossAxis) && std::isfinite(calibration.radius) &&
      std::isfinite(calibration.normalizedResidual);
  if (!calibration.valid || !valuesAreFinite) {
    return false;
  }

  const float transformDeterminant =
      calibration.scaleX * calibration.scaleY - calibration.crossAxis * calibration.crossAxis;
  const bool transformIsPositive =
      calibration.scaleX >= 0.1F && calibration.scaleY >= 0.1F && transformDeterminant >= 0.01F;
  const bool transformIsBounded = calibration.scaleX <= 10.0F && calibration.scaleY <= 10.0F &&
                                  std::fabs(calibration.crossAxis) <= 10.0F;
  const bool radiusIsValid = calibration.radius >= kMinFieldRadiusMicroTesla;
  const bool residualIsValid = calibration.normalizedResidual >= 0.0F &&
                               calibration.normalizedResidual < kMaxCalibrationResidual;
  return transformIsPositive && transformIsBounded && radiusIsValid && residualIsValid;
}

bool saveLevel(const compass::LevelCalibration& calibration) {
  const bool calibrationIsAcceptable = isLevelCalibrationAcceptable(calibration);
  if (!calibrationIsAcceptable) {
    return false;
  }
  Preferences preferences;
  if (!preferences.begin(kLevelNamespace, false)) {
    return false;
  }
  const std::size_t written = preferences.putBytes("cal", &calibration, sizeof(calibration));
  // 本体を書き切った後に版を確定し、中途半端な更新を現行値として読まない。
  const std::size_t versionWritten =
      written == sizeof(calibration) ? preferences.putUInt("version", kLevelStorageVersion) : 0;
  preferences.end();
  return written == sizeof(calibration) && versionWritten == sizeof(kLevelStorageVersion);
}

void loadLevel() {
  Preferences preferences;
  if (!preferences.begin(kLevelNamespace, true)) {
    return;
  }
  const bool hasCurrentVersion = preferences.getUInt("version", 0) == kLevelStorageVersion;
  compass::LevelCalibration stored;
  const bool hasStoredCalibration = preferences.getBytesLength("cal") == sizeof(stored);
  const bool canLoadCalibration = hasCurrentVersion && hasStoredCalibration;
  if (canLoadCalibration) {
    const std::size_t read = preferences.getBytes("cal", &stored, sizeof(stored));
    const bool readComplete = read == sizeof(stored);
    const bool storedCalibrationIsAcceptable = readComplete && isLevelCalibrationAcceptable(stored);
    if (storedCalibrationIsAcceptable) {
      g_level = stored;
    }
  }
  preferences.end();
}

const char* locationSourceName(LocationSource source) {
  switch (source) {
  case LocationSource::Fixed:
    return "fixed";
  case LocationSource::CachedGeoIp:
    return "cache";
  case LocationSource::LiveGeoIp:
    return "geoip";
  }
  return "unknown";
}

void initializeHostName() {
  std::array<std::uint8_t, 6> macAddress{};
  const esp_err_t macRead = esp_read_mac(macAddress.data(), ESP_MAC_WIFI_STA);
  const bool macReadSucceeded = macRead == ESP_OK;
  if (!macReadSucceeded) {
    // 読出し失敗時も有効なRFC 6762ラベルを保つ。通常はこの経路へ来ない。
    macAddress.fill(0);
  }
  g_hostName = connectivity::stackchanHostName(macAddress);
}

void loadClockFromRtc() {
  const bool clockAlreadyValid = g_timeValid;
  if (clockAlreadyValid) {
    return;
  }

  const auto rtcDate = M5.Rtc.getDateTime();
  const bool rtcYearIsPlausible = rtcDate.date.year >= 2024;
  if (!rtcYearIsPlausible) {
    return;
  }

  std::tm timeInfo = {};
  timeInfo.tm_year = rtcDate.date.year - 1900;
  timeInfo.tm_mon = rtcDate.date.month - 1;
  timeInfo.tm_mday = rtcDate.date.date;
  timeInfo.tm_hour = rtcDate.time.hours;
  timeInfo.tm_min = rtcDate.time.minutes;
  timeInfo.tm_sec = rtcDate.time.seconds;
  // RTC は UTC で持つ。timegm がないので mktime のエポック差で補正する。
  const std::time_t asLocal = std::mktime(&timeInfo);
  std::tm probe = {};
  probe.tm_year = 70;
  probe.tm_mon = 0;
  probe.tm_mday = 1;
  const std::time_t epochOffset = std::mktime(&probe);
  const std::time_t utc = asLocal - epochOffset;
  const bool rtcTimeIsPlausible = utc > 1700000000;
  if (!rtcTimeIsPlausible) {
    return;
  }

  timeval now = {utc, 0};
  settimeofday(&now, nullptr);
  g_timeValid = true;
}

bool loadStoredGeoIp() {
  const bool geoIpDisabled = !GEOIP_ENABLED;
  if (geoIpDisabled) {
    return false;
  }

  Preferences preferences;
  const bool opened = preferences.begin(kGeoIpNamespace, true);
  if (!opened) {
    return false;
  }
  const bool hasCurrentVersion = preferences.getUInt("version", 0) == kGeoIpStorageVersion;
  const bool hasStoredLocation = preferences.getBytesLength("location") == sizeof(g_storedGeoIp);
  const bool canRead = hasCurrentVersion && hasStoredLocation;
  std::size_t bytesRead = 0;
  if (canRead) {
    bytesRead = preferences.getBytes("location", &g_storedGeoIp, sizeof(g_storedGeoIp));
  }
  preferences.end();

  const bool readComplete = bytesRead == sizeof(g_storedGeoIp);
  const bool coordinatesAreValid = connectivity::isValidLocation(
      g_storedGeoIp.latitudeDegrees, g_storedGeoIp.longitudeEastDegrees);
  g_hasStoredGeoIp = readComplete && coordinatesAreValid;
  if (!g_hasStoredGeoIp) {
    return false;
  }

  g_observer.latitudeDegrees = g_storedGeoIp.latitudeDegrees;
  g_observer.longitudeEastDegrees = g_storedGeoIp.longitudeEastDegrees;
  g_locationSource = LocationSource::CachedGeoIp;
  return true;
}

bool saveStoredGeoIp(const connectivity::GeoIpLocation& location, std::int64_t updatedUnixSeconds) {
  const bool locationIsValid =
      location.valid &&
      connectivity::isValidLocation(location.latitudeDegrees, location.longitudeEastDegrees);
  if (!locationIsValid) {
    return false;
  }

  StoredGeoIpLocation stored;
  stored.latitudeDegrees = location.latitudeDegrees;
  stored.longitudeEastDegrees = location.longitudeEastDegrees;
  stored.updatedUnixSeconds = updatedUnixSeconds;

  Preferences preferences;
  const bool opened = preferences.begin(kGeoIpNamespace, false);
  if (!opened) {
    return false;
  }
  const std::size_t written = preferences.putBytes("location", &stored, sizeof(stored));
  const std::size_t versionWritten =
      written == sizeof(stored) ? preferences.putUInt("version", kGeoIpStorageVersion) : 0;
  preferences.end();
  const bool writeComplete =
      written == sizeof(stored) && versionWritten == sizeof(kGeoIpStorageVersion);
  if (!writeComplete) {
    return false;
  }

  g_storedGeoIp = stored;
  g_hasStoredGeoIp = true;
  return true;
}

bool storedGeoIpIsFresh(std::int64_t nowUnixSeconds) {
  const bool timestampIsOrdered =
      g_hasStoredGeoIp && nowUnixSeconds >= g_storedGeoIp.updatedUnixSeconds;
  if (!timestampIsOrdered) {
    return false;
  }
  return nowUnixSeconds - g_storedGeoIp.updatedUnixSeconds < kGeoIpRefreshSeconds;
}

void onNtpTimeSynchronized(struct timeval*) { g_ntpSyncPending.store(true); }

bool applyPendingNtpTime() {
  const bool syncWasPending = g_ntpSyncPending.exchange(false);
  if (!syncWasPending) {
    return false;
  }

  const std::time_t now = std::time(nullptr);
  const bool timeIsPlausible = now > 1700000000;
  if (!timeIsPlausible) {
    return false;
  }

  // SNTPはシステム時計を先に更新する。RTCへも同じ秒を残してオフライン起動へ渡す。
  const bool clockPersisted = setClockFromUnix(now);
  g_ntpSynchronized = clockPersisted;
  return clockPersisted;
}

void startWifi() {
  initializeHostName();
  const bool wifiIsConfigured = std::strlen(WIFI_SSID) > 0;
  if (!wifiIsConfigured) {
    return;
  }

  // EspressifのAPI要件どおり、STAを開始する前にDHCP/mDNSホスト名を設定する。
  WiFi.setHostname(g_hostName.data());
  WiFi.setAutoReconnect(true);
  // Why not setup内で接続完了を待つ: 圏外時に顔と方位制御の起動が数十秒止まる。
  // begin後はloopから接続、mDNS、NTP、GeoIPを段階的に成立させる。
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  g_wifiConnected = WiFi.status() == WL_CONNECTED;
}

void sendControlStatus() {
  std::array<char, 320> payload{};
  std::snprintf(
      payload.data(), payload.size(),
      "{\"host\":\"%s.local\",\"target\":%d,\"targetName\":\"%s\","
      "\"phase\":\"%s\",\"autoCycle\":%s,"
      "\"calibrationMaxTiltDegrees\":%.1f,\"calibrationSamples\":%u,"
      "\"calibrationCoverage\":%.2f}",
      g_hostName.data(), static_cast<int>(g_state.target), astro::targetName(g_state.target),
      app::phaseName(g_state.phase), g_state.autoCycleEnabled ? "true" : "false",
      static_cast<double>(g_calibrationTuning.maxTiltDegrees),
      static_cast<unsigned>(g_calibrationCount), static_cast<double>(g_calibrationCoverage));
  g_controlServer.sendHeader("Cache-Control", "no-store");
  g_controlServer.send(200, "application/json; charset=utf-8", payload.data());
}

void sendSensorSnapshot() {
  const std::uint32_t now = millis();
  const std::uint32_t magnetometerAgeMillis = g_hasRawMagSample ? now - g_lastMagChangeMillis : now;
  const std::uint32_t gyroscopeAgeMillis =
      g_lastGyroReadMillis > 0 ? now - g_lastGyroReadMillis : now;
  const compass::Vec3 corrected = correctedHorizontalField(g_lastRawMag, g_lastAccel);
  const float magneticHeading = compass::headingDegreesFromHorizontal(corrected);
  const float actualYawDegrees = static_cast<float>(g_lastServoAngles.yawDeci) / 10.0F;
  const float actualPitchDegrees = static_cast<float>(g_lastServoAngles.pitchDeci) / 10.0F;
  const float neckAzimuthDegrees =
      compass::normalizeDegrees(g_state.bodyHeadingDegrees + actualYawDegrees);
  const double neckAltitudeDegrees = pointing::altitudeFromPitchDeci(g_lastServoAngles.pitchDeci);

  std::array<char, 2048> payload{};
  const int written = std::snprintf(
      payload.data(), payload.size(),
      "{\"host\":\"%s.local\",\"uptimeMillis\":%lu,\"phase\":\"%s\","
      "\"target\":{\"index\":%d,\"azimuthDegrees\":%.2f,\"altitudeDegrees\":%.2f},"
      "\"magnetometer\":{\"core\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"face\":{\"xForward\":%.3f,\"yRight\":%.3f,\"zUp\":%.3f},"
      "\"averageFace\":{\"xForward\":%.3f,\"yRight\":%.3f,\"zUp\":%.3f},"
      "\"horizontalCorrected\":{\"xForward\":%.3f,\"yRight\":%.3f},"
      "\"ageMillis\":%lu,\"freshSamples\":%lu,\"repeatedSamples\":%lu},"
      "\"accelerometer\":{\"core\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},"
      "\"face\":{\"xForward\":%.4f,\"yRight\":%.4f,\"zUp\":%.4f}},"
      "\"gyroscope\":{\"core\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
      "\"face\":{\"xForward\":%.3f,\"yRight\":%.3f,\"zUp\":%.3f},"
      "\"magnitudeDegreesPerSecond\":%.3f,\"ageMillis\":%lu},"
      "\"heading\":{\"valid\":%s,\"bodyTrueDegrees\":%.2f,"
      "\"stateBodyDegrees\":%.2f,\"filteredMagneticDegrees\":%.2f,"
      "\"instantMagneticDegrees\":%.2f,\"declinationEastDegrees\":%.2f},"
      "\"servo\":{\"settled\":%s,\"actualYawDegrees\":%.1f,"
      "\"actualPitchDegrees\":%.1f,\"commandedYawDegrees\":%.1f,"
      "\"commandedPitchDegrees\":%.1f,\"neckAzimuthDegrees\":%.2f,"
      "\"neckAltitudeDegrees\":%.2f},"
      "\"calibration\":{\"valid\":%s,\"offsetX\":%.3f,\"offsetY\":%.3f,"
      "\"scaleX\":%.5f,\"scaleY\":%.5f,\"crossAxis\":%.5f,"
      "\"radius\":%.3f,\"normalizedResidual\":%.5f}}",
      g_hostName.data(), static_cast<unsigned long>(now), app::phaseName(g_state.phase),
      static_cast<int>(g_state.target), g_state.lastPosition.horizontal.azimuthDegrees,
      g_state.lastPosition.horizontal.altitudeDegrees, static_cast<double>(g_lastCoreMag.x),
      static_cast<double>(g_lastCoreMag.y), static_cast<double>(g_lastCoreMag.z),
      static_cast<double>(g_lastSensorMag.x), static_cast<double>(g_lastSensorMag.y),
      static_cast<double>(g_lastSensorMag.z), static_cast<double>(g_lastRawMag.x),
      static_cast<double>(g_lastRawMag.y), static_cast<double>(g_lastRawMag.z),
      static_cast<double>(corrected.x), static_cast<double>(corrected.y),
      static_cast<unsigned long>(magnetometerAgeMillis),
      static_cast<unsigned long>(g_magnetometerChangeCount),
      static_cast<unsigned long>(g_magnetometerRepeatedCount),
      static_cast<double>(g_lastCoreAccel.x), static_cast<double>(g_lastCoreAccel.y),
      static_cast<double>(g_lastCoreAccel.z), static_cast<double>(g_lastObservedAccel.x),
      static_cast<double>(g_lastObservedAccel.y), static_cast<double>(g_lastObservedAccel.z),
      static_cast<double>(g_lastCoreGyro.x), static_cast<double>(g_lastCoreGyro.y),
      static_cast<double>(g_lastCoreGyro.z), static_cast<double>(g_lastFaceGyro.x),
      static_cast<double>(g_lastFaceGyro.y), static_cast<double>(g_lastFaceGyro.z),
      static_cast<double>(g_lastGyroMagnitude), static_cast<unsigned long>(gyroscopeAgeMillis),
      g_headingValid ? "true" : "false", static_cast<double>(g_bodyTrueHeading),
      static_cast<double>(g_state.bodyHeadingDegrees),
      static_cast<double>(g_headingFilter.valueDegrees()), static_cast<double>(magneticHeading),
      static_cast<double>(kSiteDeclinationEast), g_lastTickServoSettled ? "true" : "false",
      static_cast<double>(actualYawDegrees), static_cast<double>(actualPitchDegrees),
      static_cast<double>(g_commandedYaw) / 10.0, static_cast<double>(g_commandedPitch) / 10.0,
      static_cast<double>(neckAzimuthDegrees), neckAltitudeDegrees,
      g_level.valid ? "true" : "false", static_cast<double>(g_level.offsetX),
      static_cast<double>(g_level.offsetY), static_cast<double>(g_level.scaleX),
      static_cast<double>(g_level.scaleY), static_cast<double>(g_level.crossAxis),
      static_cast<double>(g_level.radius), static_cast<double>(g_level.normalizedResidual));
  const bool payloadFits = written >= 0 && static_cast<std::size_t>(written) < payload.size();
  if (!payloadFits) {
    g_controlServer.send(500, "application/json", "{\"error\":\"sensor payload overflow\"}");
    return;
  }

  g_controlServer.sendHeader("Cache-Control", "no-store");
  g_controlServer.send(200, "application/json; charset=utf-8", payload.data());
}

void resetCalibrationSession() {
  // 既存のNVS結果は消さない。試行が失敗しても再起動すれば以前の補正へ戻せる。
  g_level = compass::LevelCalibration{};
  g_calibrationCount = 0;
  g_calibrationCoverage = 0.0F;
  g_calibrationCandidateRadius = 0.0F;
  g_calibrationCandidateResidual = 1.0F;
  g_calibrationFitCount = 0;
  g_calibrationTiltResetCount = 0;
  g_referenceGravity = compass::Vec3{};
  g_headingFilter.reset();
  g_headingValid = false;
  g_state.phase = app::Phase::Calibrating;
  g_state.phaseEnteredMillis = millis();
}

void handleCalibrationRequest() {
  const bool hasMaximumTilt = g_controlServer.hasArg("maxTiltDegrees");
  if (!hasMaximumTilt) {
    g_controlServer.send(400, "application/json", "{\"error\":\"maxTiltDegrees is required\"}");
    return;
  }

  const String rawMaximumTilt = g_controlServer.arg("maxTiltDegrees");
  char* parseEnd = nullptr;
  const float parsedMaximumTilt = std::strtof(rawMaximumTilt.c_str(), &parseEnd);
  const bool parsedEntireValue = parseEnd != rawMaximumTilt.c_str() && *parseEnd == '\0';
  const bool valueIsFinite = std::isfinite(parsedMaximumTilt);
  const bool valueIsInRange = parsedMaximumTilt >= 1.0F && parsedMaximumTilt <= 30.0F;
  const bool maximumTiltIsValid = parsedEntireValue && valueIsFinite && valueIsInRange;
  if (!maximumTiltIsValid) {
    g_controlServer.send(400, "application/json", "{\"error\":\"maxTiltDegrees must be 1..30\"}");
    return;
  }

  constexpr float kPi = 3.14159265358979323846F;
  g_calibrationTuning.maxTiltDegrees = parsedMaximumTilt;
  g_calibrationTuning.minimumGravityDot = std::cos(parsedMaximumTilt * kPi / 180.0F);
  resetCalibrationSession();

  std::array<char, 96> payload{};
  std::snprintf(payload.data(), payload.size(),
                "{\"accepted\":true,\"maxTiltDegrees\":%.1f,\"samples\":0}",
                static_cast<double>(g_calibrationTuning.maxTiltDegrees));
  g_controlServer.sendHeader("Cache-Control", "no-store");
  g_controlServer.send(202, "application/json; charset=utf-8", payload.data());
}

void handleTargetRequest() {
  const bool hasTarget = g_controlServer.hasArg("target");
  if (!hasTarget) {
    g_controlServer.send(400, "application/json", "{\"error\":\"target is required\"}");
    return;
  }

  const String targetValue = g_controlServer.arg("target");
  char* parseEnd = nullptr;
  const long parsedTarget = std::strtol(targetValue.c_str(), &parseEnd, 10);
  const bool parsedEntireValue = parseEnd != targetValue.c_str() && *parseEnd == '\0';
  const bool targetIsInRange = parsedTarget >= 0 && parsedTarget < kTargetCount;
  if (!parsedEntireValue || !targetIsInRange) {
    g_controlServer.send(400, "application/json", "{\"error\":\"target must be 0..7\"}");
    return;
  }

  g_requestedTarget.store(static_cast<int>(parsedTarget));
  std::array<char, 48> payload{};
  std::snprintf(payload.data(), payload.size(), "{\"accepted\":true,\"target\":%ld}", parsedTarget);
  g_controlServer.sendHeader("Cache-Control", "no-store");
  g_controlServer.send(202, "application/json", payload.data());
}

void startControlServer() {
  if (g_controlServerStarted) {
    return;
  }

  g_controlServer.on("/api/status", HTTP_GET, sendControlStatus);
  g_controlServer.on("/api/sensors", HTTP_GET, sendSensorSnapshot);
  g_controlServer.on("/api/calibration", HTTP_POST, handleCalibrationRequest);
  g_controlServer.on("/api/target", HTTP_POST, handleTargetRequest);
  g_controlServer.onNotFound(
      []() { g_controlServer.send(404, "application/json", "{\"error\":\"not found\"}"); });
  g_controlServer.begin();
  g_controlServerStarted = true;
}

bool configureNtpSynchronization() {
  const bool wifiIsConnected = WiFi.status() == WL_CONNECTED;
  if (!wifiIsConnected) {
    return false;
  }

  g_ntpSyncPending.store(false);
  sntp_set_time_sync_notification_cb(onNtpTimeSynchronized);
  configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
  g_ntpConfigured = true;
  return true;
}

connectivity::GeoIpLocation fetchGeoIpLocation() {
  connectivity::GeoIpLocation result;
  const bool canUseTls = g_timeValid && WiFi.status() == WL_CONNECTED;
  if (!canUseTls) {
    return result;
  }

  BuiltInCertificateBundleClient client;
  client.useBuiltInCertificateBundle();
  client.setHandshakeTimeout(5);
  HTTPClient request;
  request.setConnectTimeout(kGeoIpHttpTimeoutMillis);
  request.setTimeout(kGeoIpHttpTimeoutMillis);
  request.setUserAgent("compass-stackchan/1");
  const bool requestStarted = request.begin(client, GEOIP_URL);
  if (!requestStarted) {
    return result;
  }

  const int status = request.GET();
  const bool responseSucceeded = status == HTTP_CODE_OK;
  if (!responseSucceeded) {
    request.end();
    return result;
  }

  const String payload = request.getString();
  request.end();
  constexpr std::size_t kMaximumGeoIpResponseBytes = 512;
  const bool responseSizeIsSafe = payload.length() <= kMaximumGeoIpResponseBytes;
  if (!responseSizeIsSafe) {
    return result;
  }
  return connectivity::parseGeoIpResponse(std::string_view(payload.c_str(), payload.length()));
}

bool refreshGeoIpLocation() {
  const bool geoIpEnabled = GEOIP_ENABLED;
  const bool networkIsReady = g_wifiConnected && g_timeValid;
  if (!geoIpEnabled || !networkIsReady) {
    return false;
  }

  g_geoIpAttempted = true;
  g_lastGeoIpAttemptMillis = millis();
  const std::int64_t now = static_cast<std::int64_t>(std::time(nullptr));
  const bool cachedLocationIsFresh = storedGeoIpIsFresh(now);
  if (cachedLocationIsFresh) {
    return true;
  }

  const connectivity::GeoIpLocation location = fetchGeoIpLocation();
  if (!location.valid) {
    return false;
  }

  g_observer.latitudeDegrees = location.latitudeDegrees;
  g_observer.longitudeEastDegrees = location.longitudeEastDegrees;
  g_locationSource = LocationSource::LiveGeoIp;
  const bool locationSaved = saveStoredGeoIp(location, now);
  if (!locationSaved) {
    Serial.println("geoip-save=failed");
  }
  return true;
}

void serviceConnectivity(std::uint32_t nowMillis) {
  applyPendingNtpTime();

  const bool wifiIsConfigured = std::strlen(WIFI_SSID) > 0;
  if (!wifiIsConfigured) {
    return;
  }

  g_wifiConnected = WiFi.status() == WL_CONNECTED;
  if (!g_wifiConnected) {
    return;
  }

  startControlServer();
  g_controlServer.handleClient();

  const bool mdnsRetryDue =
      !g_mdnsAttempted || nowMillis - g_lastMdnsAttemptMillis >= kNetworkRetryMillis;
  const bool shouldStartMdns = !g_mdnsStarted && mdnsRetryDue;
  if (shouldStartMdns) {
    const bool shouldResetPartialMdns = g_mdnsAttempted;
    if (shouldResetPartialMdns) {
      MDNS.end();
    }
    g_mdnsAttempted = true;
    g_lastMdnsAttemptMillis = nowMillis;
    g_mdnsStarted = MDNS.begin(g_hostName.data());
    if (g_mdnsStarted) {
      MDNS.addService("stackchan", "tcp", kControlHttpPort);
      MDNS.addServiceTxt("stackchan", "tcp", "path", "/api/target");
      MDNS.addServiceTxt("stackchan", "tcp", "calibration", "/api/calibration");
    }
  }

  const bool shouldConfigureNtp = !g_ntpConfigured;
  if (shouldConfigureNtp) {
    configureNtpSynchronization();
  }

  const bool geoIpEnabled = GEOIP_ENABLED;
  if (!geoIpEnabled) {
    return;
  }
  const bool geoIpRetryDue =
      !g_geoIpAttempted || nowMillis - g_lastGeoIpAttemptMillis >= kNetworkRetryMillis;
  const bool controlIsIdle = g_state.phase == app::Phase::Tracking && g_lastTickServoSettled;
  const bool shouldRefreshGeoIp = geoIpRetryDue && controlIsIdle;
  if (shouldRefreshGeoIp) {
    refreshGeoIpLocation();
  }
}

// RTCで即時起動し、Wi-FiがあればNTPで補正してからGeoIPを更新する。
//
// Why not RTCが有効ならreturnする: RTCの経年誤差が永久に補正されず、Wi-Fi対応でも
// 天体時刻が徐々にずれる。RTCはオフライン用の初期値、NTPはオンライン時の正本。
void syncTime() {
  // BUILD_UNIX_TIMEを明示した外部ビルドでは、その値を起動時の予備時刻にする。
  // 通常のjust set-timeは書き込み時間を予測せず、起動後にUSBで現在時刻を送る。
#ifdef BUILD_UNIX_TIME
  {
    Preferences stamp;
    const bool opened = stamp.begin("clock", false);
    if (opened) {
      const int storedBuild = stamp.getInt("build", 0);
      const bool buildTimeIsNew = storedBuild != BUILD_UNIX_TIME;
      if (buildTimeIsNew) {
        const std::time_t buildTime = BUILD_UNIX_TIME;
        const bool clockSet = setClockFromUnix(buildTime);
        if (clockSet) {
          stamp.putInt("build", BUILD_UNIX_TIME);
        }
      }
      stamp.end();
    }
  }
#endif

  loadClockFromRtc();
  loadStoredGeoIp();
  startWifi();
}

// 実際の首の角度。UART 越しなのでコストが高く、両軸を一度に間隔を空けて読む。
//
// Why not 指令値: Tracking 中は deadband 内なら move() を呼ばないので、
// 指令値は古い角度のまま取り残される。ゲートが姿勢を誤判定して測定を弾く。
ServoAngles currentServoAngles(std::uint32_t nowMillis) {
  static ServoAngles cached;
  static std::uint32_t lastReadMillis = 0;
  constexpr std::uint32_t kReadIntervalMillis = 200;

  const bool readDue = nowMillis - lastReadMillis >= kReadIntervalMillis;
  if (readDue) {
    lastReadMillis = nowMillis;
    const auto angles = M5StackChan.Motion.getCurrentAngles();
    cached.yawDeci = angles.x;
    cached.pitchDeci = angles.y;
  }
  g_lastServoAngles = cached;
  return cached;
}

// 首が動いているかを、実際の角度の変化で判定する。
//
// Why not 指令からの経過時間: 指令のたび一定時間を無条件で動作中とすると、
// 首が既に目標にいる場合まで動作中扱いになり、磁気が一切採用されない。
bool servoLikelyMoving(std::uint32_t nowMillis, const ServoAngles& actual) {
  static int lastYaw = 0;
  static int lastPitch = pointing::kPitchLevelDeci;
  static std::uint32_t lastChangeMillis = 0;

  // 保持中の微振動を「動いている」と誤判定しない幅。
  //
  // Why not 5 (0.5 度): サーボは臨界減衰で目標へ漸近するので、静止して見えても
  // 数度の範囲で揺れ続ける。実測の静定精度は 8.7 度あった。0.5 度で判定すると
  // 永久に動作中と見なされ、磁気が一切採用されない (実機で発生)。
  constexpr int kMovementThresholdDeci = 30;
  const bool yawMoved = std::abs(actual.yawDeci - lastYaw) > kMovementThresholdDeci;
  const bool pitchMoved = std::abs(actual.pitchDeci - lastPitch) > kMovementThresholdDeci;
  const bool eitherAxisMoved = yawMoved || pitchMoved;
  if (eitherAxisMoved) {
    lastYaw = actual.yawDeci;
    lastPitch = actual.pitchDeci;
    lastChangeMillis = nowMillis;
    return true;
  }

  // 磁場が落ち着くまでの実測値 300ms に余裕を持たせる
  constexpr std::uint32_t kQuietMillis = 500;
  return nowMillis - lastChangeMillis < kQuietMillis;
}

// サーボへ指令を出す。
//
// 首が既に目標にいて、かつ前回と同じ指令なら何もしない。両方を見るのは、
// 指令値だけだと起動直後 (g_commandedYaw = 0) に測定姿勢の指令と一致して
// move() が一度も出ず、到達済みだけだと次に動かすべき角度を見逃すため。
// 同じ指令の再発行は servoLikelyMoving() を真に戻し、フィルタが溜まらなくなる。
void commandServo(int yawDeci, int pitchDeci, const ServoAngles& actual) {
  const bool alreadyThere = pointing::isServoAtTarget(actual.yawDeci, actual.pitchDeci, yawDeci,
                                                      pitchDeci, kServoArrivalToleranceDeci);
  const bool unchanged = yawDeci == g_commandedYaw && pitchDeci == g_commandedPitch;
  if (alreadyThere && unchanged) {
    return;
  }

  // 測定姿勢から離れるなら、溜めた方位を捨てる。首が動いた後に古い平均が
  // 残っていると、次に正面へ戻ったとき汚れた値が即座に採用されてしまう。
  if (!compass::isMeasurementPose(yawDeci)) {
    g_headingFilter.reset();
    g_headingValid = false;
  }

  g_commandedYaw = yawDeci;
  g_commandedPitch = pitchDeci;
  M5StackChan.Motion.move(yawDeci, pitchDeci, kMoveSpeed);
}

void applyServoIntent(const app::ServoIntent& intent, const ServoAngles& actual) {
  static app::Phase lastPhase = app::Phase::Error;
  const bool phaseChanged = g_state.phase != lastPhase;
  lastPhase = g_state.phase;

  // 首がまだ目標に着いていないなら、局面が続いていても指令を出し直す。
  //
  // Why not 局面が変わった瞬間だけ出す: 局面は出たり入ったりを繰り返す。
  // 実機では ReturningToMeasurePose → Measuring → (姿勢で棄却) → Idle →
  // ReturningToMeasurePose と 34ms 周期で回り、指令が首に届く前に局面が
  // 変わって、首が -98 度に取り残されたままになった。
  //
  // commandServo() 側が「到達済みかつ同じ指令なら何もしない」ので、
  // 無駄な再発行にはならない。
  const bool atTarget =
      pointing::isServoAtTarget(actual.yawDeci, actual.pitchDeci, intent.yawDeciDegrees,
                                intent.pitchDeciDegrees, kServoArrivalToleranceDeci);
  const bool notThereYet = !atTarget;

  if (intent.shouldMove || phaseChanged || notThereYet) {
    commandServo(intent.yawDeciDegrees, intent.pitchDeciDegrees, actual);
  }
}

app::Input readInput() {
  static bool readingClock = false;
  static char clockDigits[16] = {};
  static std::size_t clockDigitCount = 0;

  while (Serial.available() > 0) {
    const int serialCommand = Serial.read();

    if (readingClock) {
      const bool isDigit = serialCommand >= '0' && serialCommand <= '9';
      const bool hasRoom = clockDigitCount + 1 < sizeof(clockDigits);
      if (isDigit && hasRoom) {
        clockDigits[clockDigitCount++] = static_cast<char>(serialCommand);
        continue;
      }

      const bool isLineEnd = serialCommand == '\n' || serialCommand == '\r';
      const bool hasClockValue = clockDigitCount > 0;
      if (isLineEnd && hasClockValue) {
        clockDigits[clockDigitCount] = '\0';
        const auto requested = static_cast<std::time_t>(std::strtoll(clockDigits, nullptr, 10));
        const bool clockSet = setClockFromUnix(requested);
        if (clockSet) {
          Serial.printf("clock=%lld\n", static_cast<long long>(requested));
        } else {
          Serial.println("clock=invalid");
        }
      }
      readingClock = false;
      clockDigitCount = 0;
      if (isLineEnd) {
        continue;
      }
    }

    const bool startsClockCommand = serialCommand == 'T';
    if (startsClockCommand) {
      readingClock = true;
      clockDigitCount = 0;
      continue;
    }

    const bool requestsForward = serialCommand == '>';
    if (requestsForward) {
      return app::Input::SwipeForward;
    }
    const bool requestsBackward = serialCommand == '<';
    if (requestsBackward) {
      return app::Input::SwipeBackward;
    }
    const bool requestsClick = serialCommand == ' ';
    if (requestsClick) {
      return app::Input::Click;
    }
  }

  // 触れられているかどうかを、ジェスチャ判定とは別に記録する。
  // スワイプが効かないとき、センサに届いていないのか、ジェスチャとして
  // 認識されていないのかを切り分けるため。
  const auto& intensities = M5StackChan.TouchSensor.getIntensities();
  g_touchIntensity = static_cast<int>(intensities[0]) * 100 +
                     static_cast<int>(intensities[1]) * 10 + static_cast<int>(intensities[2]);
  if (g_touchIntensity != 0) {
    g_touchSeenCount++;
  }

  if (M5StackChan.TouchSensor.wasSwipedForward()) {
    ++g_touchEventCount;
    return app::Input::SwipeForward;
  }
  if (M5StackChan.TouchSensor.wasSwipedBackward()) {
    ++g_touchEventCount;
    return app::Input::SwipeBackward;
  }
  if (M5StackChan.TouchSensor.wasClicked()) {
    ++g_touchEventCount;
    return app::Input::Click;
  }
  return app::Input::None;
}

void readNetworkTargetSelection(app::Tick& tick) {
  const int requestedTarget = g_requestedTarget.exchange(-1);
  const bool targetIsInRange = requestedTarget >= 0 && requestedTarget < kTargetCount;
  if (!targetIsInRange) {
    return;
  }

  tick.targetSelectionRequested = true;
  tick.requestedTarget = static_cast<astro::Target>(requestedTarget);
}

// 磁気を 1 サンプル取り込む。
//
// 首が正面にないときの値はフィルタに入れない。ゲートは「採用するか」しか
// 見ないので、首を振っている間の値を溜めると、正面に戻った頃には平均が
// 汚染されていて、ゲートを通った瞬間に誤った方位を採用してしまう。
void updateHeading(const ServoAngles& actual, std::uint32_t nowMillis, bool servoInertiaSettled) {
  const bool hasFatalError = g_state.phase == app::Phase::Error;
  if (hasFatalError) {
    return;
  }
  if ((M5.Imu.update() & m5::IMU_Class::sensor_mask_mag) == 0) {
    return;
  }

  const compass::Vec3 sensorRaw = readMag();
  const compass::Vec3 accel = readAccel();
  g_lastObservedAccel = accel;
  const bool rawMagMatchesPrevious = g_hasRawMagSample && sensorRaw.x == g_lastSensorMag.x &&
                                     sensorRaw.y == g_lastSensorMag.y &&
                                     sensorRaw.z == g_lastSensorMag.z;
  if (rawMagMatchesPrevious) {
    ++g_magnetometerRepeatedCount;
    return;
  }

  g_hasRawMagSample = true;
  g_lastSensorMag = sensorRaw;
  const compass::Vec3 raw = averageMagneticSample(sensorRaw);
  g_lastRawMag = raw;
  g_lastAccel = accel;
  g_lastMagChangeMillis = nowMillis;
  ++g_magnetometerChangeCount;
  if (g_state.phase == app::Phase::Calibrating) {
    // サーボの動作中と停止直後は、顔側IMUのジャイロも磁気も動く。ジャイロの
    // 回転判定より先に棄却し、本体を回した動きだけを校正へ入れる。
    if (!servoInertiaSettled) {
      return;
    }

    const float rotationRateDegreesPerSecond = readGyroMagnitude();
    const bool bodyIsRotating =
        rotationRateDegreesPerSecond >= kCalibrationMinimumRotationRateDegreesPerSecond;
    if (!bodyIsRotating) {
      return;
    }

    // 姿勢が変わっていないサンプルだけを採る。
    //
    // Why not 水平 (accel.z がほぼ 1g) を要求する: スタックチャンの CoreS3 は
    // 顔として見やすいよう筐体が傾いており、実機では accel.z = 0.61g
    // (約 52 度傾き) で固定されていた。水平を要求すると 1 点も採れない。
    //
    // 大事なのは絶対的な水平ではなく、回している間に姿勢が変わらないこと。
    // 傾きが一定なら、磁石との相対関係も一定に保たれ、円が描ける。
    const float accelMagnitude = compass::magnitude(accel);
    const bool hasUsableGravity = accelMagnitude > 0.5F;
    if (!hasUsableGravity) {
      return;
    }
    const compass::Vec3 gravity = {accel.x / accelMagnitude, accel.y / accelMagnitude,
                                   accel.z / accelMagnitude};
    // 基準の姿勢は、サンプルを集め始めた時点のもの。
    //
    // Why not 起動時の値を持ち続ける: キャリブレーションをやり直しても最初の値が
    // 残り、書き込み直後のたまたまの姿勢が基準になる。実機では傾き 1 度以内に
    // 絞ったつもりで Z が 90uT 動いていた。集め直すたびに取り直す。
    if (g_calibrationCount == 0) {
      g_referenceGravity = gravity;
    }
    // 重力ベクトル全体のなす角で見る。Zだけでは、同じ傾斜角のまま横へ
    // 倒れた姿勢を区別できず、別の回転面を同じ円へ混ぜてしまう。
    const float gravityDot = gravity.x * g_referenceGravity.x + gravity.y * g_referenceGravity.y +
                             gravity.z * g_referenceGravity.z;
    const bool attitudeHeld = gravityDot >= g_calibrationTuning.minimumGravityDot;
    if (!attitudeHeld) {
      // 回転中にケーブルをまたぐ程度の一時的な傾きでは、それまで集めた円周を
      // 失わない。不一致点だけを捨て、円への当てはまりで集合全体を最終判定する。
      // Why not 集合を全消去: 実機では一周中に100回以上の小さな傾きが入り、
      // 毎回消すと回し終えても静止位置の4点しか残らなかった。
      ++g_calibrationTiltResetCount;
      return;
    }

    const compass::Attitude attitude = compass::attitudeFromAccel(accel);
    const compass::Vec3 horizontal = compass::horizontalMagneticComponents(raw, attitude);
    bool pointIsDistinct = true;
    for (std::size_t index = 0; index < g_calibrationCount; ++index) {
      const compass::Vec3& existing = g_calibrationSamples[index];
      const float distance = std::hypot(horizontal.x - existing.x, horizontal.y - existing.y);
      const bool pointIsNearExisting =
          distance < g_calibrationTuning.minimumPointDistanceMicroTesla;
      if (pointIsNearExisting) {
        pointIsDistinct = false;
        break;
      }
    }
    if (!pointIsDistinct) {
      return;
    }

    const bool hasSampleStorage = g_calibrationSamples != nullptr;
    const bool hasSampleCapacity = g_calibrationCount < kCalibrationCapacity;
    if (hasSampleStorage && hasSampleCapacity) {
      g_calibrationSamples[g_calibrationCount++] = horizontal;
    }
    return;
  }

  // 位置が止まった直後も、顔側IMUには首の慣性振動が残る。
  // 磁気フィルタとバイアス学習のどちらにも、その期間の値を入れない。
  if (!servoInertiaSettled) {
    return;
  }

  const compass::Vec3 corrected = correctedHorizontalField(raw, accel);
  const float measured = compass::headingDegreesFromHorizontal(corrected);

  // 横向きではyaw角だけでなくpitchと磁場強度まで変わる。実測モデルなしに
  // 補間せず、同じ正面・水平姿勢で採ったサンプルだけを平均する。
  const bool atMeasurePose = compass::isMeasurementPose(actual.yawDeci);
  if (!atMeasurePose) {
    return;
  }
  g_headingFilter.update(measured);
}

// キャリブレーションの完了判定。当てはめが重いので間隔を空けて呼ぶこと。
void tryFinishCalibration() {
  const compass::LevelCalibration calibration =
      compass::fitLevelCircle(g_calibrationSamples, g_calibrationCount);
  ++g_calibrationFitCount;
  g_calibrationCandidateRadius = calibration.radius;
  g_calibrationCandidateResidual = calibration.normalizedResidual;
  g_calibrationCoverage = calibration.valid
                              ? 1.0F
                              : compass::angularCoverage(g_calibrationSamples, g_calibrationCount,
                                                         calibration.offsetX, calibration.offsetY);
  const bool calibrationAccepted = isLevelCalibrationAcceptable(calibration);
  if (!calibrationAccepted) {
    const bool bufferIsFull = g_calibrationCount >= kCalibrationCapacity;
    if (bufferIsFull) {
      // Why not 古い点へ上書き: 異なる回転面を同じ円へ混ぜると中心が実在しない
      // 位置へ寄る。集合ごと捨て、次のサンプルを新しい重力基準にする。
      g_calibrationCount = 0;
      g_calibrationCoverage = 0.0F;
    }
    return;
  }

  g_level = calibration;
  const bool levelSaved = saveLevel(calibration);
  if (!levelSaved) {
    Serial.println("level-save=failed");
  }
  g_headingFilter.reset();
}

m5avatar::Avatar g_avatar;

// 状態を顔に反映する。
//
// 描画そのものは Avatar が自前のスレッドで回すので、ここでは表情・視線・
// 吹き出しを設定するだけにする。M5.Display へ直接書くと Avatar の描画と
// 取り合いになって画面が壊れる。
void updateFace() {
  // 吹き出しは毎周期渡すとちらつくので、変わったときだけ差し替える。
  static char lastSpeech[64] = {};
  const app::FacePresentation presentation =
      app::facePresentationFor(g_state, g_calibrationCoverage);

  switch (presentation.mood) {
  case app::FaceMood::Doubt:
    g_avatar.setExpression(m5avatar::Expression::Doubt);
    break;
  case app::FaceMood::Sleepy:
    g_avatar.setExpression(m5avatar::Expression::Sleepy);
    break;
  case app::FaceMood::Happy:
    g_avatar.setExpression(m5avatar::Expression::Happy);
    break;
  }

  const bool speechChanged = std::strcmp(presentation.speech.data(), lastSpeech) != 0;
  if (speechChanged) {
    std::snprintf(lastSpeech, sizeof(lastSpeech), "%s", presentation.speech.data());
    g_avatar.setSpeechText(presentation.speech.data());
  }

  // 指せていない分だけ視線を寄せる。首が届かないことが表情で分かる。
  float horizontalGaze = 0.0F;
  if (g_state.hasSolve && g_state.lastSolve.command.clampedYaw) {
    horizontalGaze = std::max(
        -1.0F, std::min(1.0F, static_cast<float>(g_state.lastSolve.unreachableYawDegrees) / 90.0F));
  }
  float verticalGaze = 0.0F;
  if (g_state.hasSolve && g_state.lastSolve.command.clampedPitch) {
    // Avatar の vertical は下が正。上を向くほど負にする。
    verticalGaze = std::max(
        -1.0F,
        std::min(1.0F, static_cast<float>(-g_state.lastSolve.unreachablePitchDegrees) / 45.0F));
  }
  g_avatar.setRightGaze(verticalGaze, horizontalGaze);
  g_avatar.setLeftGaze(verticalGaze, horizontalGaze);
}

} // namespace

void setup() {
  g_bootResetReason = esp_reset_reason();
  auto config = M5.config();
  M5.begin(config);
  // Why not BMI270のAUX経由でBMM150を再設定する: 実機で手動書込み後に
  // 連続読出しが止まった。M5Unifiedの初期化を保ち、ノイズはRAM上で平均する。
  // 動作中の状態を読む唯一の手段。NVS はフラッシュ読み出しに esptool のリセットを
  // 伴うので、何度読んでも「起動直後」しか観測できず、指した結果が見えない。
  Serial.begin(115200);
  M5.Display.setRotation(1);

  M5StackChan.begin();
  // 指した姿勢を保つために必須。既定では静止 200ms でトルクが切れて首が垂れる。
  M5StackChan.Motion.setAutoTorqueReleaseEnabled(false);
  M5StackChan.Motion.setAutoAngleSyncEnabled(false);

  g_observer.latitudeDegrees = kSiteLatitudeDegrees;
  g_observer.longitudeEastDegrees = kSiteLongitudeEastDegrees;

  // CoreS3 は 8MB の PSRAM を積んでいるので、サンプルを全点保持できる。
  g_calibrationSamples =
      static_cast<compass::Vec3*>(ps_malloc(kCalibrationCapacity * sizeof(compass::Vec3)));
  if (g_calibrationSamples == nullptr) {
    // PSRAM が取れないままキャリブレーションに入ると null を辿って落ちる。
    // 内部 RAM へ落として、点数を削ってでも動かす。
    g_calibrationSamples =
        static_cast<compass::Vec3*>(malloc(kCalibrationCapacity * sizeof(compass::Vec3)));
  }

  loadLevel();
  syncTime();

  // 顔。以後 M5.Display へ直接書かない (Avatar が自前のスレッドで描くため)。
  g_avatar.init();
  // 吹き出しは日本語を出すので、既定の ASCII フォントでは化ける。
  // Avatar の吹き出しは右寄せの固定位置で文字倍率も2固定。右端は
  // 220 + textWidth + 2 なので、8pxフォントの全角6文字 (96px) までなら
  // 320px画面に収まる。主語つき状態文も6文字以内に揃えている。
  g_avatar.setSpeechFont(&fonts::lgfxJapanGothicP_8);

  const bool needsCalibration = !g_level.valid;
  if (needsCalibration) {
    const bool canCollectCalibration = g_calibrationSamples != nullptr;
    g_state.phase = canCollectCalibration ? app::Phase::Calibrating : app::Phase::Error;
    g_calibrationCount = 0;
  }
}

void loop() {
  M5StackChan.update();
  const std::uint32_t now = millis();
  serviceConnectivity(now);

  const ServoAngles actual = currentServoAngles(now);
  const int actualYaw = actual.yawDeci;
  const bool servoMoving = servoLikelyMoving(now, actual);
  if (servoMoving) {
    g_lastObservedServoMotionMillis = now;
  }
  // servoLikelyMoving() 自体が位置変化後500msを待つ。さらに同じ時間を置き、
  // 位置が止まった後の慣性振動をセンサー判断へ混ぜない。
  const bool servoInertiaSettled =
      !servoMoving && now - g_lastObservedServoMotionMillis >= g_config.gyroAfterServoSettleMillis;
  updateHeading(actual, now, servoInertiaSettled);

  static std::uint32_t lastFitMillis = 0;
  const bool shouldTryFit = g_state.phase == app::Phase::Calibrating &&
                            g_calibrationCount >= kMinCalibrationSamples &&
                            now - lastFitMillis >= kFitIntervalMillis;
  if (shouldTryFit) {
    lastFitMillis = now;
    tryFinishCalibration();
  }

  compass::MeasurementGate::Input gateInput;
  gateInput.nowMillis = now;
  // 慣性待ちの間はサーボ動作中として先に棄却する。ジャイロ値を0にするのは、
  // 読んでから無視するのではなく、この期間は判断材料にしないため。
  gateInput.servoMoving = !servoInertiaSettled;
  gateInput.gyroMagnitudeDegPerSec = servoInertiaSettled ? readGyroMagnitude() : 0.0F;
  // updateHeading()と同じ磁気・加速度サンプルを使う。別々に読み直すと、巨大な
  // 固定磁場を傾斜補正した値と分散判定の値が食い違い、原因を追えない。
  const compass::Vec3 gateField = correctedHorizontalField(g_lastRawMag, g_lastAccel);
  gateInput.fieldMagnitudeMicroTesla = compass::magnitude(gateField);
  gateInput.headingDispersionDegrees = g_headingFilter.dispersionDegrees();
  gateInput.yawDeciDegrees = actualYaw;

  // 停止した瞬間を 1 回だけ記録する。ここへ来るまでに慣性待ちを済ませているので、
  // その待ち時間を差し引き、ゲート側で同じ待ちを重ねない。
  static bool wasMoving = false;
  if (wasMoving && !gateInput.servoMoving) {
    g_lastServoStopMillis = now - g_config.gyroAfterServoSettleMillis;
  }
  wasMoving = gateInput.servoMoving;
  gateInput.lastServoStopMillis = g_lastServoStopMillis;

  // 指数移動平均は最初の数サンプルが初期値に引きずられる。収束前の値を
  // 採用すると方位が大きく揺れる。
  constexpr std::size_t kMinimumSamples = 8;
  const compass::MeasurementGate::Reject reject = g_gate.evaluate(gateInput);
  const bool measurementAccepted = reject == compass::MeasurementGate::Reject::None;
  const bool headingConverged = g_headingFilter.hasConverged(kMinimumSamples);
  const bool atMeasurementPose = compass::isMeasurementPose(actualYaw);
  const bool canLearnReference =
      measurementAccepted && headingConverged && atMeasurementPose && g_level.valid;
  if (canLearnReference) {
    // 異常値を先に基準へ混ぜると、近づけた磁石そのものへ追従してしまう。
    // ゲートを通った正面サンプルだけで、校正後の水平磁場強度を学ぶ。
    g_gate.learnReferenceField(gateInput.fieldMagnitudeMicroTesla);
  }
  if (measurementAccepted && headingConverged) {
    g_bodyTrueHeading =
        compass::trueHeadingFromMagnetic(g_headingFilter.valueDegrees(), kSiteDeclinationEast);
    g_headingValid = true;
  }

  app::Tick tick;
  tick.nowMillis = now;
  tick.unixSeconds = static_cast<std::int64_t>(std::time(nullptr));
  tick.timeValid = g_timeValid;
  tick.calibrationValid = g_level.valid;
  tick.input = readInput();
  readNetworkTargetSelection(tick);
  tick.bodyTrueHeadingDegrees = g_bodyTrueHeading;
  tick.headingValid = g_headingValid;
  tick.measurementAccepted = measurementAccepted;
  tick.lastReject = reject;
  tick.gyroMagnitudeDegPerSec = gateInput.gyroMagnitudeDegPerSec;
  // State側も同じ慣性待ちを単体で保証できるよう、ここでは元のサーボ静止だけを渡す。
  // ジャイロ値自体はservoInertiaSettledまで読まないため、二重に待つことはない。
  tick.servoSettled = !servoMoving;
  g_lastTickServoSettled = servoInertiaSettled;

  app::step(g_state, tick, g_config, g_observer);
  if (g_state.headingResetRequested) {
    // 本体の方位が変わったため、移動中まで含む古い平均を破棄する。
    // 次の正面・水平測定が8サンプル収束するまで旧方位を再利用しない。
    g_headingFilter.reset();
    g_headingValid = false;
  }
  applyServoIntent(app::servoIntentFor(g_state), actual);

  // 進捗をホストから確認するための最小限の記録。
  //
  // Why not ここで当てはめ直す: 段階 8 では採用されない理由を追うために
  // fitLevelCircle と fitEllipsoid をこの場で回し、生サンプル 96 個も書いていた。
  // 512 点の最小二乗と固有値分解を NVS 書き込みと同じ周期で回すと 1 回が長すぎて
  // ウォッチドッグを踏み、起動 0.5 秒後に再起動し続けた。当てはめの結果は
  // すでに変数にあるので、読むだけにする。
  static std::uint32_t lastProbeMillis = 0;
  if (now - lastProbeMillis >= kProbeIntervalMillis) {
    lastProbeMillis = now;
    // 首が実際に指している方位 = 機体の向き + 首の相対角。
    // 目標を指せているかを、目視でなく数値で検証する。
    //
    // 機体の向きは state 側の採用済みの値を使う。g_bodyTrueHeading は首を振ると
    // 更新が止まるので、指した直後の検証に使うと古い値が混ざる。
    const int yawDeci = actual.yawDeci;
    const int pitchDeci = actual.pitchDeci;
    const float neckAbsolute = g_state.bodyHeadingDegrees + static_cast<float>(yawDeci) / 10.0F;
    const double neckAltitude = pointing::altitudeFromPitchDeci(pitchDeci);

    // NVS はフラッシュなので、シリアルと同じ頻度で書くと摩耗する。
    static std::uint32_t lastNvsMillis = 0;
    Preferences probe;
    const bool shouldWriteNvs = now - lastNvsMillis >= kNvsProbeIntervalMillis;
    if (shouldWriteNvs && probe.begin("probe", false)) {
      lastNvsMillis = now;
      probe.putInt("calN", static_cast<int>(g_calibrationCount));
      // 水平補正の採否。read_nvs.py はスカラーしか読まないので blob とは別に置く。
      probe.putInt("lvOk", g_level.valid ? 1 : 0);
      probe.putInt("lvCov", static_cast<int>(std::lround(g_calibrationCoverage * 100.0F)));
      probe.putInt("hdgOk", g_headingValid ? 1 : 0);
      probe.putInt("hdg", static_cast<int>(std::lround(g_bodyTrueHeading * 10.0F)));
      probe.putInt("phase", static_cast<int>(g_state.phase));
      probe.putInt("rej", static_cast<int>(g_state.lastReject));
      // Pointing に入ってからの経過。抜けない理由の切り分け用。
      probe.putInt("inPhase", static_cast<int>(now - g_state.phaseEnteredMillis));
      probe.putInt("settled", g_lastTickServoSettled ? 1 : 0);
      probe.putInt("yaw", yawDeci);
      probe.putInt("pitch", pitchDeci);
      probe.putInt("cmdYaw", g_commandedYaw);
      probe.putInt("cmdPitch", g_commandedPitch);
      probe.putInt("neckAbs", static_cast<int>(std::lround(neckAbsolute * 10.0F)));
      probe.putInt("neckAlt", static_cast<int>(std::lround(neckAltitude * 10.0)));
      probe.putInt("tgtAz", static_cast<int>(std::lround(
                                g_state.lastPosition.horizontal.azimuthDegrees * 10.0F)));
      probe.putInt("eph", static_cast<int>(g_state.lastPosition.source));
      probe.putInt("clampY", g_state.lastSolve.command.clampedYaw ? 1 : 0);
      probe.putInt("clampP", g_state.lastSolve.command.clampedPitch ? 1 : 0);
      // 時刻が入っているかと、いま指しているターゲット。
      probe.putInt("timeOk", g_timeValid ? 1 : 0);
      probe.putInt("target", static_cast<int>(g_state.target));
      // タッチ入力が届いているか。スワイプが効かないときの切り分け用。
      probe.putInt("touchN", g_touchEventCount);
      probe.end();
    }

    // 同じ内容をシリアルへも出す。NVS は起動直後しか読めないので、
    // 動いている最中の検証はこちらを使う。
    //
    // nS と settled は「rej=0 なのに hdgOk=0」を切り分けるために要る。
    // ゲートを通っていても、フィルタが溜まっていなければ方位は確定しない。
    Serial.printf(
        "phase=%s up=%lu rst=%d target=%d rej=%d hdgOk=%d hdg=%.1f yaw=%d pitch=%d "
        "cmdYaw=%d cmdPitch=%d neckAbs=%.1f neckAlt=%.1f tgtAz=%.1f alt=%.1f "
        "eph=%s clampY=%d clampP=%d timeOk=%d touchN=%d calN=%u calCov=%.2f "
        "calRad=%.1f calErr=%.3f calFit=%lu calTilt=%lu calMax=%.1f nS=%u "
        "mean=%.1f disp=%.1f field=%.1f ref=%.1f mx=%.1f my=%.1f mz=%.1f "
        "ax=%.3f ay=%.3f az=%.3f ox=%.1f oy=%.1f sx=%.3f sy=%.3f cx=%.3f "
        "rad=%.1f err=%.3f magAvg=%u magAge=%lu magFreshN=%lu magRepeatN=%lu "
        "wifi=%d mdns=%d api=%d ntp=%d host=%s loc=%s lat=%.4f lon=%.4f "
        "settled=%d gyro=%.1f motionN=%lu force=%d tI=%03d tSeen=%d\n",
        app::phaseName(g_state.phase), static_cast<unsigned long>(now),
        static_cast<int>(g_bootResetReason), static_cast<int>(g_state.target),
        static_cast<int>(g_state.lastReject), g_headingValid ? 1 : 0,
        static_cast<double>(g_bodyTrueHeading), yawDeci, pitchDeci, g_commandedYaw,
        g_commandedPitch, static_cast<double>(neckAbsolute), neckAltitude,
        g_state.lastPosition.horizontal.azimuthDegrees,
        g_state.lastPosition.horizontal.altitudeDegrees,
        astro::ephemerisSourceName(g_state.lastPosition.source),
        g_state.lastSolve.command.clampedYaw ? 1 : 0,
        g_state.lastSolve.command.clampedPitch ? 1 : 0, g_timeValid ? 1 : 0, g_touchEventCount,
        static_cast<unsigned>(g_calibrationCount), static_cast<double>(g_calibrationCoverage),
        static_cast<double>(g_calibrationCandidateRadius),
        static_cast<double>(g_calibrationCandidateResidual),
        static_cast<unsigned long>(g_calibrationFitCount),
        static_cast<unsigned long>(g_calibrationTiltResetCount),
        static_cast<double>(g_calibrationTuning.maxTiltDegrees),
        static_cast<unsigned>(g_headingFilter.sampleCount()),
        static_cast<double>(g_headingFilter.valueDegrees()),
        static_cast<double>(g_headingFilter.dispersionDegrees()),
        static_cast<double>(gateInput.fieldMagnitudeMicroTesla),
        static_cast<double>(g_gate.referenceFieldMicroTesla()), static_cast<double>(g_lastRawMag.x),
        static_cast<double>(g_lastRawMag.y), static_cast<double>(g_lastRawMag.z),
        static_cast<double>(g_lastObservedAccel.x), static_cast<double>(g_lastObservedAccel.y),
        static_cast<double>(g_lastObservedAccel.z), static_cast<double>(g_level.offsetX),
        static_cast<double>(g_level.offsetY), static_cast<double>(g_level.scaleX),
        static_cast<double>(g_level.scaleY), static_cast<double>(g_level.crossAxis),
        static_cast<double>(g_level.radius), static_cast<double>(g_level.normalizedResidual),
        static_cast<unsigned>(kMagneticAverageWindow),
        static_cast<unsigned long>(g_hasRawMagSample ? now - g_lastMagChangeMillis : now),
        static_cast<unsigned long>(g_magnetometerChangeCount),
        static_cast<unsigned long>(g_magnetometerRepeatedCount), g_wifiConnected ? 1 : 0,
        g_wifiConnected && g_mdnsStarted ? 1 : 0, g_wifiConnected && g_controlServerStarted ? 1 : 0,
        g_ntpSynchronized ? 1 : 0, g_hostName.data(), locationSourceName(g_locationSource),
        g_observer.latitudeDegrees, g_observer.longitudeEastDegrees, g_lastTickServoSettled ? 1 : 0,
        static_cast<double>(gateInput.gyroMagnitudeDegPerSec),
        static_cast<unsigned long>(g_state.bodyMotionCount), g_state.forceMeasurementPose ? 1 : 0,
        g_touchIntensity, g_touchSeenCount);
  }

  static std::uint32_t lastDraw = 0;
  if (now - lastDraw >= 200) {
    lastDraw = now;
    updateFace();
  }

  delay(10);
}
