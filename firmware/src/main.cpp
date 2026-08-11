// compass-stackchan 本体。
//
// 真北と天体 (太陽・月・水星〜土星) の方向を、首で物理的に指し示す。
// yaw で方位を、pitch で高度を表す。
//
// 磁気測定は必ず首を正面に戻してから行う。首の角度によって方位が最大 119 度
// ずれることを実機で確認しているため (段階 8)。ロジックは app_core 側にあり、
// ここは実機の入出力を state machine に橋渡しするだけ。
//
// 状態はシリアルへ出す (just watch / just verify で読む)。
//
// Why not NVS: フラッシュの読み出しは esptool のリセットを伴うので、何度読んでも
// 「起動直後」の値しか取れず、指した結果が観測できない。NVS には節目の値だけを
// 30 秒間隔で残す。毎周期フラッシュへ書き、そこで球の当てはめまで回していた頃は、
// それ自体がウォッチドッグを踏んで数秒ごとに再起動していた。

#include <M5StackChan.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <WiFi.h>

#include "app/state.hpp"
#include "compass/calibration.hpp"
#include "compass/declination.hpp"
#include "compass/heading.hpp"
#include "compass/level_calibration.hpp"
#include "compass/sphere_fit.hpp"
#include "compass/stability.hpp"
#include "location_config.h"

#include <algorithm>
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
constexpr std::size_t kMinCalibrationSamples = 24;
constexpr float kMinFieldRadiusMicroTesla = 12.0F;
// 補正後に球へどれだけ乗っていれば採用するか。実機のハードアイアンは地磁気の
// 6 倍あり歪みが強いので、厳しくしすぎると永久に採用されない。
constexpr float kMaxCalibrationResidual = 0.25F;
// 当てはめは数百点の最小二乗で重い。毎周期回すとウォッチドッグを踏む。
constexpr std::uint32_t kFitIntervalMillis = 700;
// 状態をシリアルへ出す間隔。局面の遷移が追える程度に細かくする。
// NVS への書き込みはこれより間引く (フラッシュの摩耗を避けるため)。
constexpr std::uint32_t kProbeIntervalMillis = 500;
constexpr std::uint32_t kNvsProbeIntervalMillis = 30000;

app::State g_state;
app::Config g_config;
astro::Observer g_observer;

compass::MagCalibration g_calibration;
// 3x3 のソフトアイアン補正。実機の歪みは対角では直せず、方位が 32 度ずれた。
compass::EllipsoidFit g_ellipsoid;
// 水平回転で取る補正。この機体は本体に強い磁石があり、傾けると磁石も一緒に
// 動くので 8 の字回しでは地磁気の球にならない (実機で半径 169uT = 地磁気の
// 3.7 倍になった)。首を固定して本体だけ水平に回せば相対関係が保たれる。
compass::LevelCalibration g_level;
// 直近の当てはめの被覆率。描画から読むだけにして、当てはめは間隔を空けて回す。
float g_calibrationCoverage = 0.0F;
// キャリブレーション開始時の姿勢。ここから外れたサンプルは採らない。
float g_referenceTilt = 0.0F;
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

void saveLevel(const compass::LevelCalibration& calibration) {
  Preferences preferences;
  if (!preferences.begin("level", false)) {
    return;
  }
  preferences.putBytes("cal", &calibration, sizeof(calibration));
  preferences.end();
}

void loadLevel() {
  Preferences preferences;
  if (!preferences.begin("level", true)) {
    return;
  }
  compass::LevelCalibration stored;
  if (preferences.getBytesLength("cal") == sizeof(stored)) {
    preferences.getBytes("cal", &stored, sizeof(stored));
    g_level = stored;
  }
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

// 時刻を用意する。優先順位は RTC → Wi-Fi + NTP。
//
// CoreS3 は RTC を積んでいるので、一度合わせれば電源を切っても保たれる。
// Wi-Fi が無い場所でも天体を指せるようにするため、まず RTC を見る。
void syncTime() {
  // ホストのビルド時刻で RTC を合わせる。
  //
  // Wi-Fi の無い場所でも天体を指せるようにするための手段。just set-time で
  // 書き込むと、そのビルドの時刻が RTC に入る。数秒の誤差は天体の方位に
  // 換算して 0.001 度未満なので、この用途には十分。
#ifdef BUILD_UNIX_TIME
  {
    Preferences stamp;
    if (stamp.begin("clock", false)) {
      const int storedBuild = stamp.getInt("build", 0);
      if (storedBuild != BUILD_UNIX_TIME) {
        stamp.putInt("build", BUILD_UNIX_TIME);
        const std::time_t buildTime = BUILD_UNIX_TIME;
        const std::tm* utc = std::gmtime(&buildTime);
        if (utc != nullptr) {
          m5::rtc_datetime_t rtc;
          rtc.date.year = static_cast<std::uint16_t>(utc->tm_year + 1900);
          rtc.date.month = static_cast<std::uint8_t>(utc->tm_mon + 1);
          rtc.date.date = static_cast<std::uint8_t>(utc->tm_mday);
          rtc.time.hours = static_cast<std::uint8_t>(utc->tm_hour);
          rtc.time.minutes = static_cast<std::uint8_t>(utc->tm_min);
          rtc.time.seconds = static_cast<std::uint8_t>(utc->tm_sec);
          M5.Rtc.setDateTime(rtc);
        }
      }
      stamp.end();
    }
  }
#endif

  // RTC に妥当な時刻が入っていれば、それを使う。
  auto rtcDate = M5.Rtc.getDateTime();
  if (rtcDate.date.year >= 2024) {
    std::tm timeInfo = {};
    timeInfo.tm_year = rtcDate.date.year - 1900;
    timeInfo.tm_mon = rtcDate.date.month - 1;
    timeInfo.tm_mday = rtcDate.date.date;
    timeInfo.tm_hour = rtcDate.time.hours;
    timeInfo.tm_min = rtcDate.time.minutes;
    timeInfo.tm_sec = rtcDate.time.seconds;
    // RTC は UTC で持つ。timegm がないので mktime との差で補正する。
    const std::time_t asLocal = std::mktime(&timeInfo);
    std::tm probe = {};
    probe.tm_year = 70;
    probe.tm_mon = 0;
    probe.tm_mday = 1;
    const std::time_t epochOffset = std::mktime(&probe);
    const std::time_t utc = asLocal - epochOffset;
    if (utc > 1700000000) {
      timeval now = {utc, 0};
      settimeofday(&now, nullptr);
      g_timeValid = true;
      return;
    }
  }

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
      // 次回は Wi-Fi 無しでも動くよう RTC に残す。
      const std::time_t now = std::time(nullptr);
      const std::tm* utc = std::gmtime(&now);
      if (utc != nullptr) {
        m5::rtc_datetime_t rtc;
        rtc.date.year = static_cast<std::uint16_t>(utc->tm_year + 1900);
        rtc.date.month = static_cast<std::uint8_t>(utc->tm_mon + 1);
        rtc.date.date = static_cast<std::uint8_t>(utc->tm_mday);
        rtc.time.hours = static_cast<std::uint8_t>(utc->tm_hour);
        rtc.time.minutes = static_cast<std::uint8_t>(utc->tm_min);
        rtc.time.seconds = static_cast<std::uint8_t>(utc->tm_sec);
        M5.Rtc.setDateTime(rtc);
      }
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

  // 保持中の微振動を「動いている」と誤判定しない幅。
  //
  // Why not 5 (0.5 度): サーボは臨界減衰で目標へ漸近するので、静止して見えても
  // 数度の範囲で揺れ続ける。実測の静定精度は 8.7 度あった。0.5 度で判定すると
  // 永久に動作中と見なされ、磁気が一切採用されない (実機で発生)。
  constexpr int kMovementThresholdDeci = 30;
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

  // 首がまだ目標に着いていないなら、局面が続いていても指令を出し直す。
  //
  // Why not 局面が変わった瞬間だけ出す: 局面は出たり入ったりを繰り返す。
  // 実機では ReturningToMeasurePose → Measuring → (姿勢で棄却) → Idle →
  // ReturningToMeasurePose と 34ms 周期で回り、指令が首に届く前に局面が
  // 変わって、首が -98 度に取り残されたままになった。
  //
  // commandServo() 側が「到達済みかつ同じ指令なら何もしない」ので、
  // 無駄な再発行にはならない。
  const bool notThereYet =
      std::abs(actualYaw - intent.yawDeciDegrees) > compass::kMeasurementYawToleranceDeci;

  if (intent.shouldMove || phaseChanged || notThereYet) {
    commandServo(intent.yawDeciDegrees, intent.pitchDeciDegrees, actualYaw);
  }
}

app::Input readInput() {
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
    // 姿勢が変わっていないサンプルだけを採る。
    //
    // Why not 水平 (accel.z がほぼ 1g) を要求する: スタックチャンの CoreS3 は
    // 顔として見やすいよう筐体が傾いており、実機では accel.z = 0.61g
    // (約 52 度傾き) で固定されていた。水平を要求すると 1 点も採れない。
    //
    // 大事なのは絶対的な水平ではなく、回している間に姿勢が変わらないこと。
    // 傾きが一定なら、磁石との相対関係も一定に保たれ、円が描ける。
    const compass::Vec3 accel = readAccel();
    const float tilt = accel.z / std::max(0.001F, compass::magnitude(accel));
    // 基準の姿勢は、サンプルを集め始めた時点のもの。
    //
    // Why not static で持ち続ける: キャリブレーションをやり直しても最初の値が
    // 残り、書き込み直後のたまたまの姿勢が基準になる。実機では傾き 1 度以内に
    // 絞ったつもりで Z が 90uT 動いていた。集め直すたびに取り直す。
    if (g_calibrationCount == 0) {
      g_referenceTilt = tilt;
    }
    const float referenceTilt = g_referenceTilt;
    // 姿勢の許容を狭くする。実機では 5 度の揺れでも、補正後の半径が
    // 2〜22uT に散らばって方位誤差 23 度になった。回転面がぶれると
    // 断面の半径が変わるため、傾きに対する感度が高い。
    const bool attitudeHeld = std::fabs(tilt - referenceTilt) < 0.02F; // 約 1 度
    if (attitudeHeld && g_calibrationSamples != nullptr &&
        g_calibrationCount < kCalibrationCapacity) {
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
  const compass::LevelCalibration calibration =
      compass::fitLevelCircle(g_calibrationSamples, g_calibrationCount);
  g_calibrationCoverage = calibration.valid
                              ? 1.0F
                              : compass::angularCoverage(g_calibrationSamples, g_calibrationCount,
                                                         calibration.offsetX, calibration.offsetY);
  if (!calibration.valid || calibration.radius < kMinFieldRadiusMicroTesla) {
    return;
  }
  if (calibration.normalizedResidual >= kMaxCalibrationResidual) {
    return;
  }

  g_level = calibration;
  saveLevel(calibration);
  g_headingFilter.reset();
}

// 直近の当てはめ結果。描画から使う。
// 描画のたびに当てはめを回すとウォッチドッグを踏むので、
// tryFinishCalibration() が更新した値を読むだけにする。
void drawCalibrating() {
  auto& display = M5.Display;
  display.setTextColor(TFT_YELLOW, TFT_BLACK);
  display.setCursor(4, 40);
  display.print("spin in place ");
  display.setCursor(4, 76);
  display.printf("%d%%      ", static_cast<int>(g_calibrationCoverage * 100.0F));

  // 姿勢が変わると採れないので、その場で伝える。
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.setCursor(4, 112);
  display.printf("n=%d      ", static_cast<int>(g_calibrationCount));
}

// ターゲットごとの色。切り替わったことが一目で分かるようにする。
std::uint16_t targetColor(astro::Target target) {
  switch (target) {
  case astro::Target::North:
    return TFT_WHITE;
  case astro::Target::Sun:
    return TFT_ORANGE;
  case astro::Target::Moon:
    return TFT_SILVER;
  case astro::Target::Mercury:
    return TFT_DARKGREY;
  case astro::Target::Venus:
    return TFT_YELLOW;
  case astro::Target::Mars:
    return TFT_RED;
  case astro::Target::Jupiter:
    return TFT_ORANGE;
  case astro::Target::Saturn:
    return TFT_GOLD;
  case astro::Target::kCount:
    break;
  }
  return TFT_WHITE;
}

// 方位を 16 方位の記号にする。数値より向きが掴みやすい。
const char* compassPoint(double azimuthDegrees) {
  static const char* kPoints[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  auto index = static_cast<int>(std::lround(azimuthDegrees / 22.5)) % 16;
  if (index < 0) {
    index += 16;
  }
  return kPoints[index];
}

void drawStatus() {
  auto& display = M5.Display;

  // ターゲットが変わったら画面を消す。文字サイズが混在するので、
  // 上書きだけだと前の文字が残る。
  static astro::Target lastTarget = astro::Target::kCount;
  static bool lastWasCalibrating = true;
  const bool isCalibrating = g_state.phase == app::Phase::Calibrating;
  if (g_state.target != lastTarget || isCalibrating != lastWasCalibrating) {
    display.fillScreen(TFT_BLACK);
    lastTarget = g_state.target;
    lastWasCalibrating = isCalibrating;
  }

  if (isCalibrating) {
    display.setTextSize(2);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setCursor(4, 4);
    display.print("compass       ");
    drawCalibrating();
    return;
  }

  // ターゲット名を大きく出す。スワイプで切り替わるのが主役なので。
  display.setTextSize(3);
  display.setTextColor(targetColor(g_state.target), TFT_BLACK);
  display.setCursor(4, 4);
  display.printf("%-8s ", astro::targetName(g_state.target));

  display.setTextSize(2);

  if (!g_state.lastPosition.valid) {
    display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    display.setCursor(4, 44);
    display.print("no time sync   ");
    display.setCursor(4, 70);
    display.print("set wifi_config");
    return;
  }

  const auto& horizontal = g_state.lastPosition.horizontal;
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setCursor(4, 44);
  display.printf("%-3s %5.1f    ", compassPoint(horizontal.azimuthDegrees),
                 horizontal.azimuthDegrees);
  display.setCursor(4, 70);
  display.printf("alt %+5.1f     ", horizontal.altitudeDegrees);

  // 指せているかを伝える。黙って端に張り付くと指していると誤解される。
  display.setCursor(4, 104);
  if (g_state.lastSolve.command.clampedPitch) {
    display.setTextColor(TFT_ORANGE, TFT_BLACK);
    display.printf("%+.0f deg higher ", g_state.lastSolve.unreachablePitchDegrees);
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

  // 下段は状態。方位が無効なら赤で示す。
  display.setTextSize(1);
  display.setTextColor(g_headingValid ? TFT_DARKGREY : TFT_RED, TFT_BLACK);
  display.setCursor(4, 138);
  display.printf("hdg %5.1f  %-13s %s    ", g_bodyTrueHeading, app::phaseName(g_state.phase),
                 g_state.autoCycleEnabled ? "AUTO" : "");

  // 操作の案内。触れば分かるが、最初の一回のために出しておく。
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.setCursor(4, 152);
  display.print("swipe: target   tap: auto");
}

} // namespace

void setup() {
  auto config = M5.config();
  M5.begin(config);
  // 動作中の状態を読む唯一の手段。NVS はフラッシュ読み出しに esptool のリセットを
  // 伴うので、何度読んでも「起動直後」しか観測できず、指した結果が見えない。
  Serial.begin(115200);
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
  if (g_calibrationSamples == nullptr) {
    // PSRAM が取れないままキャリブレーションに入ると null を辿って落ちる。
    // 内部 RAM へ落として、点数を削ってでも動かす。
    g_calibrationSamples =
        static_cast<compass::Vec3*>(malloc(kCalibrationCapacity * sizeof(compass::Vec3)));
  }

  // タッチセンサ (Si12T, 0x68) が I2C に居るかを起動時に一度確かめる。
  // 実機でスワイプが 1 度も検出されなかったので、ジェスチャ判定の問題なのか
  // センサまで届いていないのかを切り分ける。
  loadCalibration();
  loadLevel();
  syncTime();

  if (!g_level.valid) {
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
  tick.calibrationValid = g_level.valid;
  tick.input = readInput();
  tick.bodyTrueHeadingDegrees = g_bodyTrueHeading;
  tick.headingValid = g_headingValid;
  tick.measurementAccepted = reject == compass::MeasurementGate::Reject::None;
  tick.lastReject = reject;
  tick.gyroMagnitudeDegPerSec = gateInput.gyroMagnitudeDegPerSec;
  tick.servoSettled = !gateInput.servoMoving;
  g_lastTickServoSettled = tick.servoSettled;

  app::step(g_state, tick, g_config, g_observer);
  applyServoIntent(app::servoIntentFor(g_state), actualYaw);

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
    const int yawDeci = currentYawDeci(now);
    const float neckAbsolute = g_state.bodyHeadingDegrees + static_cast<float>(yawDeci) / 10.0F;

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
      probe.putInt("neckAbs", static_cast<int>(std::lround(neckAbsolute * 10.0F)));
      probe.putInt("tgtAz", static_cast<int>(std::lround(
                                g_state.lastPosition.horizontal.azimuthDegrees * 10.0F)));
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
    Serial.printf("phase=%s target=%d rej=%d hdgOk=%d hdg=%.1f yaw=%d neckAbs=%.1f "
                  "tgtAz=%.1f alt=%.1f timeOk=%d touchN=%d calN=%u nS=%u settled=%d "
                  "tI=%03d tSeen=%d\n",
                  app::phaseName(g_state.phase), static_cast<int>(g_state.target),
                  static_cast<int>(g_state.lastReject), g_headingValid ? 1 : 0,
                  static_cast<double>(g_bodyTrueHeading), yawDeci,
                  static_cast<double>(neckAbsolute), g_state.lastPosition.horizontal.azimuthDegrees,
                  g_state.lastPosition.horizontal.altitudeDegrees, g_timeValid ? 1 : 0,
                  g_touchEventCount, static_cast<unsigned>(g_calibrationCount),
                  static_cast<unsigned>(g_headingFilter.sampleCount()),
                  g_lastTickServoSettled ? 1 : 0, g_touchIntensity, g_touchSeenCount);
  }

  static std::uint32_t lastDraw = 0;
  if (now - lastDraw >= 200) {
    lastDraw = now;
    drawStatus();
  }

  delay(10);
}
