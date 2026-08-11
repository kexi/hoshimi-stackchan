// compass-stackchan 本体。
//
// 真北と天体 (太陽・月・水星〜土星) の方向を、首で物理的に指し示す。
// yaw で方位を、pitch で高度を表す。
//
// 磁気測定は必ず首を正面に戻してから行う。首の角度によって方位が最大 119 度
// ずれることを実機で確認しているため (段階 8)。ロジックは app_core 側にあり、
// ここは実機の入出力を state machine に橋渡しするだけ。
//
// 診断用の NVS 書き込みは意図的に置いていない。デバッグのために毎周期
// フラッシュへ書き、球の当てはめを回していたところ、それ自体がウォッチドッグを
// 踏んで 4 秒ごとに再起動していた。状態は画面に出せば足りる。

#include <M5StackChan.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <WiFi.h>

#include "app/state.hpp"
#include "compass/calibration.hpp"
#include "compass/declination.hpp"
#include "compass/heading.hpp"
#include "compass/sphere_fit.hpp"
#include "compass/stability.hpp"
#include "location_config.h"

#include <cmath>
#include <cstdint>
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

namespace {

constexpr const char* kCalibrationNamespace = "magcal";
constexpr int kMoveSpeed = 400;
constexpr std::uint32_t kWifiTimeoutMillis = 20000;

// キャリブレーションのサンプルを全点保持して球に当てはめる。
//
// Why not min/max: 各軸の端に到達した 2 点しか使わないので、端まで回しきらないと
// 中心が「回した範囲の中心」に寄る。実機ではそれで補正後の水平成分が地磁気の
// 6 割になり、方位が全方位に散らばった。最小二乗なら球面の一部さえ掃ければ
// 中心が求まるので、短い回転で済む。
constexpr std::size_t kCalibrationCapacity = 512;
constexpr std::size_t kMinCalibrationSamples = 120;
constexpr float kMinFieldRadiusMicroTesla = 20.0F;
// 補正後に球へどれだけ乗っていれば採用するか。実機のハードアイアンは地磁気の
// 6 倍あり歪みが強いので、厳しくしすぎると永久に採用されない。
constexpr float kMaxCalibrationResidual = 0.25F;
// 当てはめは数百点の最小二乗で重い。毎周期回すとウォッチドッグを踏む。
constexpr std::uint32_t kFitIntervalMillis = 2000;

app::State g_state;
app::Config g_config;
astro::Observer g_observer;

compass::MagCalibration g_calibration;
// 3x3 のソフトアイアン補正。実機の歪みは対角では直せず、方位が 32 度ずれた。
compass::EllipsoidFit g_ellipsoid;
compass::MeasurementGate g_gate;
compass::HeadingFilter g_headingFilter{0.5F};

compass::Vec3* g_calibrationSamples = nullptr;
std::size_t g_calibrationCount = 0;

bool g_timeValid = false;
bool g_headingValid = false;
float g_bodyTrueHeading = 0.0F;

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

// 3x3 補正を NVS に入れる。行列は 9 要素あるので blob で扱う。
void saveEllipsoid(const compass::EllipsoidFit& fit) {
  Preferences preferences;
  if (!preferences.begin("ellip", false)) {
    return;
  }
  preferences.putBytes("fit", &fit, sizeof(fit));
  preferences.end();
}

void loadEllipsoid() {
  Preferences preferences;
  if (!preferences.begin("ellip", true)) {
    return;
  }
  compass::EllipsoidFit stored;
  if (preferences.getBytesLength("fit") == sizeof(stored)) {
    preferences.getBytes("fit", &stored, sizeof(stored));
    g_ellipsoid = stored;
  }
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

// 実際の首の角度。UART 越しなのでコストが高く、間隔を空けて読む。
//
// Why not 指令値: Tracking 中は deadband 内なら move() を呼ばないので、
// 指令値は古い角度のまま取り残される。ゲートが姿勢を誤判定して測定を弾く。
int currentYawDeci(std::uint32_t nowMillis) {
  static int cachedYaw = 0;
  static std::uint32_t lastReadMillis = 0;
  constexpr std::uint32_t kReadIntervalMillis = 200;

  if (nowMillis - lastReadMillis >= kReadIntervalMillis) {
    lastReadMillis = nowMillis;
    cachedYaw = M5StackChan.Motion.getCurrentYawAngle();
  }
  return cachedYaw;
}

// 首が動いているかを、実際の角度の変化で判定する。
//
// Why not 指令からの経過時間: 指令のたび一定時間を無条件で動作中とすると、
// 首が既に目標にいる場合まで動作中扱いになり、磁気が一切採用されない。
bool servoLikelyMoving(std::uint32_t nowMillis, int actualYaw) {
  static int lastYaw = 0;
  static std::uint32_t lastChangeMillis = 0;

  constexpr int kMovementThresholdDeci = 5;
  if (std::abs(actualYaw - lastYaw) > kMovementThresholdDeci) {
    lastYaw = actualYaw;
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
void commandServo(int yawDeci, int pitchDeci, int actualYaw) {
  const bool alreadyThere = std::abs(actualYaw - yawDeci) <= compass::kMeasurementYawToleranceDeci;
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
  g_lastServoCommandMillis = millis();
  M5StackChan.Motion.move(yawDeci, pitchDeci, kMoveSpeed);
}

void applyServoIntent(const app::ServoIntent& intent, int actualYaw) {
  static app::Phase lastPhase = app::Phase::Error;
  const bool phaseChanged = g_state.phase != lastPhase;
  lastPhase = g_state.phase;

  if (intent.shouldMove || phaseChanged) {
    commandServo(intent.yawDeciDegrees, intent.pitchDeciDegrees, actualYaw);
  }
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

// 磁気を 1 サンプル取り込む。
//
// 首が正面にないときの値はフィルタに入れない。ゲートは「採用するか」しか
// 見ないので、首を振っている間の値を溜めると、正面に戻った頃には平均が
// 汚染されていて、ゲートを通った瞬間に誤った方位を採用してしまう。
void updateHeading(int yawDeci, std::uint32_t nowMillis) {
  if ((M5.Imu.update() & m5::IMU_Class::sensor_mask_mag) == 0) {
    return;
  }

  const compass::Vec3 raw = readMag();
  if (g_state.phase == app::Phase::Calibrating) {
    if (g_calibrationSamples != nullptr && g_calibrationCount < kCalibrationCapacity) {
      g_calibrationSamples[g_calibrationCount++] = raw;
    }
    return;
  }

  const bool poseIsClean =
      compass::isMeasurementPose(yawDeci) && !servoLikelyMoving(nowMillis, yawDeci);
  if (!poseIsClean) {
    return;
  }

  // 3x3 が使えるならそちらを優先する。対角では実機の歪みを直しきれない。
  const compass::Vec3 corrected = g_ellipsoid.valid ? compass::applyEllipsoid(g_ellipsoid, raw)
                                                    : compass::applyCalibration(g_calibration, raw);
  const compass::Attitude attitude = compass::attitudeFromAccel(readAccel());
  g_headingFilter.update(compass::tiltCompensatedHeadingDegrees(corrected, attitude));
  g_gate.learnReferenceField(compass::magnitude(raw));
}

// キャリブレーションの完了判定。当てはめが重いので間隔を空けて呼ぶこと。
void tryFinishCalibration() {
  const compass::EllipsoidFit fit = compass::fitEllipsoid(g_calibrationSamples, g_calibrationCount);
  if (!fit.valid || fit.meanRadius <= kMinFieldRadiusMicroTesla) {
    return;
  }
  if (fit.normalizedResidual >= kMaxCalibrationResidual) {
    return;
  }

  g_ellipsoid = fit;
  saveEllipsoid(fit);

  // 対角側も埋めておく。3x3 が使えない経路 (古い保存など) の保険。
  const compass::SphereFit sphere = compass::fitSphere(g_calibrationSamples, g_calibrationCount);
  const compass::MagCalibration calibration =
      compass::calibrationFromSphere(sphere, g_calibrationSamples, g_calibrationCount);
  if (calibration.valid) {
    g_calibration = calibration;
    saveCalibration(calibration);
  }
  g_headingFilter.reset();
}

void drawCalibrating() {
  auto& display = M5.Display;
  const int percent = static_cast<int>(g_calibrationCount * 100 / kMinCalibrationSamples);
  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.setCursor(4, 40);
  display.print("turn all ways    ");
  display.setCursor(4, 76);
  display.printf("%d%%      ", percent > 100 ? 100 : percent);
}

void drawStatus() {
  auto& display = M5.Display;
  display.setTextSize(2);

  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setCursor(4, 4);
  display.printf("%-9s      ", astro::targetName(g_state.target));

  if (g_state.phase == app::Phase::Calibrating) {
    drawCalibrating();
    return;
  }

  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.setCursor(4, 30);
  display.printf("%-14s", app::phaseName(g_state.phase));

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

  // 首が届いていないなら伝える。黙って端に張り付くと指していると誤解される。
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

  display.setTextColor(g_headingValid ? TFT_DARKGREY : TFT_RED, TFT_BLACK);
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
  M5StackChan.Motion.setAutoAngleSyncEnabled(false);

  g_observer.latitudeDegrees = kSiteLatitudeDegrees;
  g_observer.longitudeEastDegrees = kSiteLongitudeEastDegrees;

  // CoreS3 は 8MB の PSRAM を積んでいるので、サンプルを全点保持できる。
  g_calibrationSamples =
      static_cast<compass::Vec3*>(ps_malloc(kCalibrationCapacity * sizeof(compass::Vec3)));

  loadCalibration();
  loadEllipsoid();
  syncTime();

  if (!g_ellipsoid.valid) {
    g_state.phase = app::Phase::Calibrating;
    g_calibrationCount = 0;
  }
}

void loop() {
  M5StackChan.update();
  const std::uint32_t now = millis();

  const int actualYaw = currentYawDeci(now);
  updateHeading(actualYaw, now);

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
  gateInput.servoMoving = servoLikelyMoving(now, actualYaw);
  gateInput.gyroMagnitudeDegPerSec = readGyroMagnitude();
  gateInput.fieldMagnitudeMicroTesla = compass::magnitude(readMag());
  gateInput.headingDispersionDegrees = g_headingFilter.dispersionDegrees();
  gateInput.yawDeciDegrees = actualYaw;

  // 停止した瞬間を 1 回だけ記録する。servoLikelyMoving() が静穏を確かめた上で
  // false を返すので、ゲート側の settle をさらに課すと待ちが二重になる。
  static bool wasMoving = false;
  if (wasMoving && !gateInput.servoMoving) {
    g_lastServoStopMillis = now - g_config.servoSettleMillis;
  }
  wasMoving = gateInput.servoMoving;
  gateInput.lastServoStopMillis = g_lastServoStopMillis;

  // 指数移動平均は最初の数サンプルが初期値に引きずられる。収束前の値を
  // 採用すると方位が大きく揺れる。
  constexpr std::size_t kMinimumSamples = 8;
  const compass::MeasurementGate::Reject reject = g_gate.evaluate(gateInput);
  if (reject == compass::MeasurementGate::Reject::None &&
      g_headingFilter.hasConverged(kMinimumSamples)) {
    g_bodyTrueHeading =
        compass::trueHeadingFromMagnetic(g_headingFilter.valueDegrees(), kSiteDeclinationEast);
    g_headingValid = true;
  }

  app::Tick tick;
  tick.nowMillis = now;
  tick.unixSeconds = static_cast<std::int64_t>(std::time(nullptr));
  tick.timeValid = g_timeValid;
  tick.calibrationValid = g_ellipsoid.valid;
  tick.input = readInput();
  tick.bodyTrueHeadingDegrees = g_bodyTrueHeading;
  tick.headingValid = g_headingValid;
  tick.measurementAccepted = reject == compass::MeasurementGate::Reject::None;
  tick.lastReject = reject;
  tick.gyroMagnitudeDegPerSec = gateInput.gyroMagnitudeDegPerSec;
  tick.servoSettled = !gateInput.servoMoving;

  app::step(g_state, tick, g_config, g_observer);
  applyServoIntent(app::servoIntentFor(g_state), actualYaw);

  // 進捗をホストから確認するための最小限の記録。
  // 診断のために毎周期フラッシュへ書いていたら 4 秒ごとに再起動した。
  // 30 秒に 1 回、値 1 つだけに留める。
  static std::uint32_t lastProbeMillis = 0;
  if (now - lastProbeMillis >= 30000) {
    lastProbeMillis = now;
    Preferences probe;
    if (probe.begin("probe", false)) {
      probe.putInt("calN", static_cast<int>(g_calibrationCount));
      // 当てはめが採用されない理由を数値で見る。30 秒に 1 回だけなので軽い。
      if (g_calibrationCount >= kMinCalibrationSamples) {
        const compass::SphereFit fit = compass::fitSphere(g_calibrationSamples, g_calibrationCount);
        probe.putInt("fitR", static_cast<int>(std::lround(fit.radius * 10.0F)));
        const compass::MagCalibration candidate =
            compass::calibrationFromSphere(fit, g_calibrationSamples, g_calibrationCount);
        probe.putInt("res", static_cast<int>(std::lround(
                                compass::residualAfterCalibration(candidate, g_calibrationSamples,
                                                                  g_calibrationCount) *
                                1000.0F)));
      }
      probe.end();
    }
  }

  static std::uint32_t lastDraw = 0;
  if (now - lastDraw >= 200) {
    lastDraw = now;
    drawStatus();
  }

  delay(10);
}
