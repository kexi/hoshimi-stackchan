// compass-stackchan 本体。
//
// 真北と天体 (太陽・月・水星〜土星) の方向を、首で物理的に指し示す。
// yaw で方位を、pitch で高度を表す。
//
// 磁気測定は必ず首を正面に戻してから行う。首の角度によって方位が最大 119 度
// ずれることを実機で確認しているため (段階 8)。ロジックは app_core 側にあり、
// ここは実機の入出力を state machine に橋渡しするだけ。

#include <M5StackChan.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <WiFi.h>

#include "app/state.hpp"
#include "compass/calibration.hpp"
#include "compass/declination.hpp"
#include "compass/heading.hpp"
#include "compass/stability.hpp"
#include "location_config.h"

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
// 実体は .gitignore 済み。無ければ Wi-Fi を使わず、真北だけを指すモードで動く。
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif

namespace {

constexpr const char* kCalibrationNamespace = "magcal";
constexpr int kMoveSpeed = 400;
constexpr std::uint32_t kWifiTimeoutMillis = 20000;

app::State g_state;
app::Config g_config;
astro::Observer g_observer;

compass::MagCalibration g_calibration;
compass::MeasurementGate g_gate;
compass::HeadingFilter g_headingFilter{0.25F};
compass::CalibrationCollector g_collector;

bool g_timeValid = false;
bool g_headingValid = false;
float g_bodyTrueHeading = 0.0F;

// 自前で持つサーボの動作状態。BSP の isMoving() は実サーボへ UART 問い合わせ
// するのでコストが高く、毎周期は呼べない。
std::uint32_t g_lastServoCommandMillis = 0;
std::uint32_t g_lastServoStopMillis = 0;
int g_commandedYaw = 0;
int g_commandedPitch = pointing::kPitchLevelDeci;

compass::Vec3 readMag() {
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

float readGyroMagnitude() {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  M5.Imu.getGyro(&x, &y, &z);
  return std::sqrt(x * x + y * y + z * z);
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

void saveCalibration(const compass::MagCalibration& calibration) {
  Preferences preferences;
  if (!preferences.begin(kCalibrationNamespace, false)) {
    return;
  }
  preferences.putInt("offX", static_cast<int>(std::lround(calibration.hardIronOffset.x * 100.0F)));
  preferences.putInt("offY", static_cast<int>(std::lround(calibration.hardIronOffset.y * 100.0F)));
  preferences.putInt("offZ", static_cast<int>(std::lround(calibration.hardIronOffset.z * 100.0F)));
  preferences.putInt("sclX", static_cast<int>(std::lround(calibration.softIronScale.x * 1000.0F)));
  preferences.putInt("sclY", static_cast<int>(std::lround(calibration.softIronScale.y * 1000.0F)));
  preferences.putInt("sclZ", static_cast<int>(std::lround(calibration.softIronScale.z * 1000.0F)));
  preferences.putInt("valid", calibration.valid ? 1 : 0);
  preferences.end();
}

// Wi-Fi と NTP。失敗しても続行する。真北は時計が無くても指せる。
void syncTime() {
  if (std::strlen(WIFI_SSID) == 0) {
    return;
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  const std::uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < kWifiTimeoutMillis) {
    delay(200);
  }
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  configTime(0, 0, "ntp.nict.jp", "pool.ntp.org");
  const std::uint32_t ntpStart = millis();
  while (millis() - ntpStart < 10000) {
    if (std::time(nullptr) > 1700000000) {
      g_timeValid = true;
      return;
    }
    delay(200);
  }
}

// サーボへ指令を出す。動作中の推定にも使う。
void commandServo(int yawDeci, int pitchDeci) {
  const bool unchanged = yawDeci == g_commandedYaw && pitchDeci == g_commandedPitch;
  if (unchanged) {
    return;
  }
  g_commandedYaw = yawDeci;
  g_commandedPitch = pitchDeci;
  g_lastServoCommandMillis = millis();
  M5StackChan.Motion.move(yawDeci, pitchDeci, kMoveSpeed);
}

// 指令からの経過で動作中かを推定する。UART 問い合わせを毎周期しないための近似。
bool servoLikelyMoving(std::uint32_t nowMillis) {
  return nowMillis - g_lastServoCommandMillis < g_config.servoSettleMillis;
}

app::Input readInput() {
  if (M5StackChan.TouchSensor.wasSwipedForward()) {
    return app::Input::SwipeForward;
  }
  if (M5StackChan.TouchSensor.wasSwipedBackward()) {
    return app::Input::SwipeBackward;
  }
  if (M5StackChan.TouchSensor.wasClicked()) {
    return app::Input::Click;
  }
  return app::Input::None;
}

// 磁気を 1 サンプル取り込む。新しい値が来ていなければ何もしない。
void updateHeading() {
  if ((M5.Imu.update() & m5::IMU_Class::sensor_mask_mag) == 0) {
    return;
  }

  const compass::Vec3 raw = readMag();
  if (g_state.phase == app::Phase::Calibrating) {
    g_collector.addSample(raw);
    return;
  }

  const compass::Vec3 corrected = compass::applyCalibration(g_calibration, raw);
  const compass::Attitude attitude = compass::attitudeFromAccel(readAccel());
  const float magneticHeading = compass::tiltCompensatedHeadingDegrees(corrected, attitude);
  g_headingFilter.update(magneticHeading);

  // 静穏時の |B| を学習する。採用できる状況のときだけ。
  if (compass::isMeasurementPose(g_commandedYaw) && !servoLikelyMoving(millis())) {
    g_gate.learnReferenceField(compass::magnitude(raw));
  }
}

void drawStatus() {
  auto& display = M5.Display;
  display.setTextSize(2);

  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(4, 4);
  display.printf("%-9s      ", astro::targetName(g_state.target));

  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.setCursor(4, 30);
  display.printf("%-14s", app::phaseName(g_state.phase));

  if (g_state.phase == app::Phase::Calibrating) {
    display.setTextColor(TFT_YELLOW, TFT_BLACK);
    display.setCursor(4, 60);
    display.printf("rotate fig-8 %d%%  ", static_cast<int>(g_collector.coverage() * 100.0F));
    return;
  }

  if (!g_state.lastPosition.valid) {
    display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    display.setCursor(4, 60);
    display.print("no time sync   ");
    return;
  }

  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setCursor(4, 60);
  display.printf("az %5.1f      ",
                 static_cast<double>(g_state.lastPosition.horizontal.azimuthDegrees));
  display.setCursor(4, 86);
  display.printf("alt %+5.1f     ",
                 static_cast<double>(g_state.lastPosition.horizontal.altitudeDegrees));

  // 首が届いていないなら、そのことを伝える。黙って端に張り付くと
  // 「指している」と誤解される。
  display.setCursor(4, 116);
  if (g_state.lastSolve.command.clampedPitch) {
    display.setTextColor(TFT_ORANGE, TFT_BLACK);
    display.printf("%+.0f deg higher  ",
                   static_cast<double>(g_state.lastSolve.unreachablePitchDegrees));
  } else if (g_state.lastSolve.command.clampedYaw) {
    display.setTextColor(TFT_ORANGE, TFT_BLACK);
    display.print("turn me around ");
  } else if (!g_state.lastPosition.aboveHorizon) {
    display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    display.print("below horizon  ");
  } else {
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.print("pointing       ");
  }

  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.setCursor(4, 146);
  display.printf("hdg %5.1f %s  ", static_cast<double>(g_bodyTrueHeading),
                 g_state.autoCycleEnabled ? "auto" : "    ");
}

} // namespace

void setup() {
  auto config = M5.config();
  M5.begin(config);
  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);

  M5StackChan.begin();
  // 指した姿勢を保つために必須。既定では静止 200ms でトルクが切れて首が垂れる。
  M5StackChan.Motion.setAutoTorqueReleaseEnabled(false);
  // 追尾中は高頻度で角度を更新するので、同期は切っておく。
  M5StackChan.Motion.setAutoAngleSyncEnabled(false);

  g_observer.latitudeDegrees = kSiteLatitudeDegrees;
  g_observer.longitudeEastDegrees = kSiteLongitudeEastDegrees;

  loadCalibration();
  syncTime();

  // キャリブレーションが無ければ 8 の字回しから始める。
  if (!g_calibration.valid) {
    g_state.phase = app::Phase::Calibrating;
    g_collector.reset();
  }
}

void loop() {
  M5StackChan.update();
  const std::uint32_t now = millis();

  updateHeading();

  // 8 の字回しの完了判定
  if (g_state.phase == app::Phase::Calibrating && g_collector.coverage() >= 1.0F) {
    const compass::MagCalibration calibration = g_collector.finish();
    if (calibration.valid) {
      g_calibration = calibration;
      saveCalibration(calibration);
      g_headingFilter.reset();
    }
  }

  // 測定ゲート。首が正面にあり、静止していて、値が安定しているときだけ採用する。
  compass::MeasurementGate::Input gateInput;
  gateInput.nowMillis = now;
  gateInput.servoMoving = servoLikelyMoving(now);
  gateInput.lastServoStopMillis = g_lastServoStopMillis;
  gateInput.gyroMagnitudeDegPerSec = readGyroMagnitude();
  gateInput.fieldMagnitudeMicroTesla = compass::magnitude(readMag());
  gateInput.headingDispersionDegrees = g_headingFilter.dispersionDegrees();
  gateInput.yawDeciDegrees = g_commandedYaw;

  if (!gateInput.servoMoving) {
    g_lastServoStopMillis = g_lastServoCommandMillis + g_config.servoSettleMillis;
  }

  const compass::MeasurementGate::Reject reject = g_gate.evaluate(gateInput);
  const bool accepted = reject == compass::MeasurementGate::Reject::None;
  if (accepted && g_headingFilter.hasValue()) {
    g_bodyTrueHeading =
        compass::trueHeadingFromMagnetic(g_headingFilter.valueDegrees(), kSiteDeclinationEast);
    g_headingValid = true;
  }

  app::Tick tick;
  tick.nowMillis = now;
  tick.unixSeconds = static_cast<std::int64_t>(std::time(nullptr));
  tick.timeValid = g_timeValid;
  tick.calibrationValid = g_calibration.valid;
  tick.input = readInput();
  tick.bodyTrueHeadingDegrees = g_bodyTrueHeading;
  tick.headingValid = g_headingValid;
  tick.measurementAccepted = accepted;
  tick.lastReject = reject;
  tick.gyroMagnitudeDegPerSec = gateInput.gyroMagnitudeDegPerSec;
  tick.servoSettled = !gateInput.servoMoving;

  app::step(g_state, tick, g_config, g_observer);

  const app::ServoIntent intent = app::servoIntentFor(g_state);
  if (intent.shouldMove) {
    commandServo(intent.yawDeciDegrees, intent.pitchDeciDegrees);
  }

  static std::uint32_t lastDraw = 0;
  if (now - lastDraw >= 200) {
    lastDraw = now;
    drawStatus();
  }

  delay(10);
}
