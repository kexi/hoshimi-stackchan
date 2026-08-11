// 段階 8: 磁気ノイズの実測。この企画の成否を分ける工程。
//
// 測るのは 3 つ:
//   1. サーボの通電状態 (電源OFF / トルクOFF / トルクON) で |B| がどれだけ変わるか
//      → 差が 5uT を超えるなら、測定中にサーボ電源を落とす層 3 が必須
//   2. yaw を 16 ビンに振ったときの方位測定値のばらつき
//      → 2 度を超えるなら、角度依存バイアス表の層 4 が必須
//   3. サーボ停止から方位が落ち着くまでの時間 → settleMillis を決める
//
// 機体は固定して動かさないこと。首だけが動く。真方位は全ビンで同一のはずなので、
// ビン間の差がそのままサーボ由来の誤差になる。
//
// 結果は NVS に書く (この個体は USB CDC が列挙されずシリアルが使えない)。

#include <M5StackChan.h>
#include <M5Unified.h>
#include <Preferences.h>

#include "compass/calibration.hpp"
#include "compass/heading.hpp"
#include "compass/stability.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>

namespace {

constexpr const char* kResultNamespace = "stage8";
constexpr const char* kCalibrationNamespace = "magcal";

constexpr int kMoveSpeed = 400;
// サーボ停止後、この時間まで方位を追いかけて収束を見る
constexpr std::uint32_t kSettleProbeMillis = 2000;

compass::MagCalibration g_calibration;

void saveInt(const char* key, int value) {
  Preferences preferences;
  if (!preferences.begin(kResultNamespace, false)) {
    return;
  }
  preferences.putInt(key, value);
  preferences.end();
}

void loadCalibration() {
  Preferences preferences;
  if (!preferences.begin(kCalibrationNamespace, true)) {
    return;
  }
  g_calibration.hardIronOffset.x = preferences.getInt("offX", 0) / 100.0F;
  g_calibration.hardIronOffset.y = preferences.getInt("offY", 0) / 100.0F;
  g_calibration.hardIronOffset.z = preferences.getInt("offZ", 0) / 100.0F;
  g_calibration.softIronScale.x = preferences.getInt("sclX", 1000) / 1000.0F;
  g_calibration.softIronScale.y = preferences.getInt("sclY", 1000) / 1000.0F;
  g_calibration.softIronScale.z = preferences.getInt("sclZ", 1000) / 1000.0F;
  g_calibration.valid = preferences.getInt("valid", 0) != 0;
  preferences.end();
}

void showLine(int y, std::uint16_t color, const char* format, ...) {
  char buffer[64];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  M5.Display.setTextColor(color, TFT_BLACK);
  M5.Display.setCursor(4, y);
  M5.Display.print(buffer);
  M5.Display.print("      ");
}

compass::Vec3 readMagRaw() {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  M5.Imu.getMag(&x, &y, &z);
  return compass::Vec3{x, y, z};
}

compass::Vec3 readAccel() {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  M5.Imu.getAccel(&x, &y, &z);
  return compass::Vec3{x, y, z};
}

// mag が更新されるまで待って 1 サンプル取る。取れなければ false。
bool waitForMagSample(compass::Vec3& magOut) {
  const std::uint32_t start = millis();
  while (millis() - start < 200) {
    if ((M5.Imu.update() & m5::IMU_Class::sensor_mask_mag) != 0) {
      magOut = readMagRaw();
      return true;
    }
    delay(2);
  }
  return false;
}

struct FieldStats {
  float meanMagnitude = 0.0F;
  float meanHeading = 0.0F;
  float headingSpread = 0.0F;
  int samples = 0;
};

// 指定時間ぶん平均を取る。方位は円環量なので sin/cos で平均する。
FieldStats sampleField(std::uint32_t durationMillis) {
  FieldStats stats;
  float magnitudeSum = 0.0F;
  float sinSum = 0.0F;
  float cosSum = 0.0F;
  float minHeading = 999.0F;
  float maxHeading = -999.0F;

  const std::uint32_t start = millis();
  while (millis() - start < durationMillis) {
    compass::Vec3 raw;
    if (!waitForMagSample(raw)) {
      continue;
    }
    magnitudeSum += compass::magnitude(raw);

    const compass::Vec3 corrected = compass::applyCalibration(g_calibration, raw);
    const compass::Attitude attitude = compass::attitudeFromAccel(readAccel());
    const float heading = compass::tiltCompensatedHeadingDegrees(corrected, attitude);
    const float radians = heading * 3.14159265F / 180.0F;
    sinSum += std::sin(radians);
    cosSum += std::cos(radians);
    minHeading = std::fmin(minHeading, heading);
    maxHeading = std::fmax(maxHeading, heading);
    ++stats.samples;
  }

  if (stats.samples == 0) {
    return stats;
  }
  stats.meanMagnitude = magnitudeSum / static_cast<float>(stats.samples);
  stats.meanHeading = std::atan2(sinSum, cosSum) * 180.0F / 3.14159265F;
  if (stats.meanHeading < 0.0F) {
    stats.meanHeading += 360.0F;
  }
  stats.headingSpread = maxHeading - minHeading;
  return stats;
}

float angleDifference(float from, float to) {
  float difference = std::fmod(to - from, 360.0F);
  if (difference > 180.0F) {
    difference -= 360.0F;
  }
  if (difference <= -180.0F) {
    difference += 360.0F;
  }
  return difference;
}

// 測定 1: サーボの通電状態による |B| の差
void measurePowerStates() {
  showLine(4, TFT_YELLOW, "1/3 power states");

  // 首は正面・水平で固定したまま、通電状態だけを変える
  M5StackChan.setServoPowerEnabled(true);
  M5StackChan.Motion.setTorqueEnabled(true);
  M5StackChan.Motion.move(0, 450, kMoveSpeed);
  delay(2000);

  const FieldStats torqueOn = sampleField(2000);
  saveInt("fieldTorqueOn", static_cast<int>(std::lround(torqueOn.meanMagnitude * 100.0F)));
  saveInt("headTorqueOn", static_cast<int>(std::lround(torqueOn.meanHeading * 10.0F)));
  showLine(30, TFT_WHITE, "on   %.1f", static_cast<double>(torqueOn.meanMagnitude));

  M5StackChan.Motion.setTorqueEnabled(false);
  delay(1500);
  const FieldStats torqueOff = sampleField(2000);
  saveInt("fieldTorqueOff", static_cast<int>(std::lround(torqueOff.meanMagnitude * 100.0F)));
  saveInt("headTorqueOff", static_cast<int>(std::lround(torqueOff.meanHeading * 10.0F)));
  showLine(56, TFT_WHITE, "toff %.1f", static_cast<double>(torqueOff.meanMagnitude));

  M5StackChan.setServoPowerEnabled(false);
  delay(1500);
  const FieldStats powerOff = sampleField(2000);
  saveInt("fieldPowerOff", static_cast<int>(std::lround(powerOff.meanMagnitude * 100.0F)));
  saveInt("headPowerOff", static_cast<int>(std::lround(powerOff.meanHeading * 10.0F)));
  showLine(82, TFT_WHITE, "poff %.1f", static_cast<double>(powerOff.meanMagnitude));

  // 層 3 の要否判定: 電源 OFF を基準に、通電時がどれだけずれるか
  const float fieldDelta = std::fabs(torqueOn.meanMagnitude - powerOff.meanMagnitude);
  const float headingDelta = std::fabs(angleDifference(powerOff.meanHeading, torqueOn.meanHeading));
  saveInt("powerFieldDelta", static_cast<int>(std::lround(fieldDelta * 100.0F)));
  saveInt("powerHeadDelta", static_cast<int>(std::lround(headingDelta * 10.0F)));
  showLine(108, fieldDelta > 5.0F ? TFT_RED : TFT_GREEN, "dB=%.1f dH=%.1f",
           static_cast<double>(fieldDelta), static_cast<double>(headingDelta));

  // 以降の測定のためにサーボを戻す
  M5StackChan.setServoPowerEnabled(true);
  M5StackChan.Motion.setTorqueEnabled(true);
  delay(1000);
}

// 測定 2: yaw 角度ごとの方位のばらつき。機体は固定なので真方位は不変のはず。
void measureYawBias() {
  M5.Display.fillScreen(TFT_BLACK);
  showLine(4, TFT_YELLOW, "2/3 yaw bias");

  constexpr int kBins = 16;
  float headings[kBins] = {};
  int validBins = 0;

  for (int bin = 0; bin < kBins; ++bin) {
    // -1280..1280 を 16 分割した各ビンの中央
    const int yawDeci = -1280 + (2560 * bin) / kBins + (2560 / kBins) / 2;
    M5StackChan.Motion.move(yawDeci, 450, kMoveSpeed);
    delay(1400);

    // トルクを切ってから測る (層 3 の効果込みの実力を見る)
    M5StackChan.Motion.setTorqueEnabled(false);
    delay(500);
    const FieldStats stats = sampleField(700);
    M5StackChan.Motion.setTorqueEnabled(true);

    if (stats.samples > 0) {
      headings[validBins++] = stats.meanHeading;
      saveInt((String("yawBin") + bin).c_str(),
              static_cast<int>(std::lround(stats.meanHeading * 10.0F)));
    }
    showLine(30, TFT_WHITE, "bin %d/%d", bin + 1, kBins);
    showLine(56, TFT_CYAN, "hdg %.1f", static_cast<double>(stats.meanHeading));
  }

  // ビン間の最大差 (円環量なので基準を bin0 にして畳む)
  float minDelta = 999.0F;
  float maxDelta = -999.0F;
  for (int bin = 0; bin < validBins; ++bin) {
    const float delta = angleDifference(headings[0], headings[bin]);
    minDelta = std::fmin(minDelta, delta);
    maxDelta = std::fmax(maxDelta, delta);
  }
  const float spread = maxDelta - minDelta;
  saveInt("yawSpread", static_cast<int>(std::lround(spread * 10.0F)));
  saveInt("yawBinsOk", validBins);

  showLine(82, spread > 2.0F ? TFT_RED : TFT_GREEN, "spread %.1f", static_cast<double>(spread));

  M5StackChan.Motion.move(0, 450, kMoveSpeed);
  delay(1500);
}

// 測定 3: サーボ停止から方位が落ち着くまでの時間
void measureSettleTime() {
  M5.Display.fillScreen(TFT_BLACK);
  showLine(4, TFT_YELLOW, "3/3 settle time");

  // 大きく振ってから止め、方位が最終値に収まるまでを追う
  M5StackChan.Motion.move(-1000, 450, kMoveSpeed);
  delay(2000);
  M5StackChan.Motion.move(1000, 450, kMoveSpeed);

  // 停止直後から 100ms 刻みで方位を記録する
  constexpr int kSlots = 20;
  float samples[kSlots] = {};
  int slotCount = 0;

  const std::uint32_t moveStart = millis();
  // move の収束を待ってから計測を始める
  delay(1200);
  const std::uint32_t stopMillis = millis();

  while (millis() - stopMillis < kSettleProbeMillis && slotCount < kSlots) {
    const FieldStats stats = sampleField(100);
    if (stats.samples > 0) {
      samples[slotCount++] = stats.meanHeading;
    }
  }
  saveInt("settleSlots", slotCount);
  saveInt("moveElapsed", static_cast<int>(millis() - moveStart));

  if (slotCount < 3) {
    showLine(30, TFT_RED, "no samples");
    return;
  }

  // 最終値から 1 度以内に入った最初の時刻を収束時刻とみなす
  const float finalHeading = samples[slotCount - 1];
  int settleSlot = slotCount - 1;
  for (int slot = 0; slot < slotCount; ++slot) {
    if (std::fabs(angleDifference(finalHeading, samples[slot])) < 1.0F) {
      settleSlot = slot;
      break;
    }
  }
  const int settleMillis = settleSlot * 100;
  saveInt("settleMillis", settleMillis);

  // 動作直後と最終値の差 = 動いている最中の誤差の大きさ
  const float initialError = std::fabs(angleDifference(finalHeading, samples[0]));
  saveInt("settleInitErr", static_cast<int>(std::lround(initialError * 10.0F)));

  showLine(30, TFT_GREEN, "settle %d ms", settleMillis);
  showLine(56, TFT_WHITE, "initErr %.1f", static_cast<double>(initialError));
}

} // namespace

void setup() {
  auto config = M5.config();
  M5.begin(config);
  M5.Display.setRotation(1);
  M5.Display.setTextSize(2);
  M5.Display.fillScreen(TFT_BLACK);

  M5StackChan.begin();
  M5StackChan.Motion.setAutoTorqueReleaseEnabled(false);
  M5StackChan.Motion.setAutoAngleSyncEnabled(false);

  loadCalibration();
  if (!g_calibration.valid) {
    showLine(4, TFT_RED, "no calibration");
    showLine(30, TFT_WHITE, "run stage7 first");
    return;
  }

  showLine(4, TFT_WHITE, "keep body still");
  delay(2000);

  measurePowerStates();
  measureYawBias();
  measureSettleTime();

  M5.Display.fillScreen(TFT_BLACK);
  showLine(4, TFT_GREEN, "=== measured ===");
  showLine(30, TFT_WHITE, "read via just");
  showLine(56, TFT_WHITE, "read-nvs");
}

void loop() {
  M5StackChan.update();
  delay(50);
}
