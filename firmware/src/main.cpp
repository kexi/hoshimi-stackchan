// 段階 6: ファーム最小構成 + 起動診断。
//
// 確かめたいこと:
//   1. framework = arduino 単独で StackChan-BSP がビルド・動作するか
//   2. pitch が上端 (900) まで実際に届くか
//      (BSP は NVS の zero_pos_2 を原点に使い、raw を 0..1000 で clamp するので、
//       個体の zero_pos 次第では上端がサチる可能性がある)
//
// 進捗は LCD にも出す。USB CDC のシリアルは、ホスト側が掴めていなかったり
// アプリが落ちていたりすると何も見えず、どこまで進んだかの判別がつかないため。
// LCD なら本体だけで確認できる。

#include <M5StackChan.h>
#include <M5Unified.h>
#include <Preferences.h>

#include "pointing/servo_map.hpp"

#include <cstdint>
#include <cstdio>

namespace {

// 首を動かしてから、スプリング応答が収まるのを待つ時間。
// BSP の speed はバネ定数へのマッピングで、臨界減衰なのでオーバーシュートはしない。
constexpr std::uint32_t kSettleMillis = 1200;
constexpr int kMoveSpeed = 400;

int g_lineY = 0;

// 計測結果は NVS に残す。USB シリアルが使えない状況でも、次回起動時に
// 読み出せば結果を回収できる (画面を人が読んで伝える必要がなくなる)。
constexpr const char* kResultNamespace = "stage6";

void saveResult(const char* key, int value) {
  Preferences preferences;
  if (!preferences.begin(kResultNamespace, false)) {
    return;
  }
  preferences.putInt(key, value);
  preferences.end();
}

// 前回の結果を画面に出す。書き込み直後の起動では前回値が、
// 2 回目以降の起動では今回の値が読める。
void showPreviousResults() {
  Preferences preferences;
  if (!preferences.begin(kResultNamespace, true)) {
    return;
  }
  const int pitchMax = preferences.getInt("pitchMax", -1);
  const int yawMax = preferences.getInt("yawMax", -1);
  const int yawMin = preferences.getInt("yawMin", -1);
  const int holdDrift = preferences.getInt("holdDrift", -1);
  preferences.end();

  if (pitchMax < 0) {
    return;
  }
  auto& display = M5.Display;
  display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  display.setCursor(4, display.height() - 18);
  display.printf("prev p%d y%d/%d d%d", pitchMax, yawMin, yawMax, holdDrift);
}

// LCD とシリアルの両方に出す。どちらか一方しか見えない状況でも追える。
void trace(const char* message) {
  Serial.println(message);

  auto& display = M5.Display;
  if (g_lineY > display.height() - 16) {
    display.fillScreen(TFT_BLACK);
    g_lineY = 0;
  }
  display.setCursor(0, g_lineY);
  display.print(message);
  g_lineY += 16;
}

void traceAngles(const char* label) {
  const int yaw = M5StackChan.Motion.getCurrentYawAngle();
  const int pitch = M5StackChan.Motion.getCurrentPitchAngle();
  char buffer[96];
  std::snprintf(buffer, sizeof(buffer), "%s y=%d p=%d", label, yaw, pitch);
  trace(buffer);
}

void moveAndReport(const char* label, int yawDeci, int pitchDeci) {
  M5StackChan.Motion.move(yawDeci, pitchDeci, kMoveSpeed);
  delay(kSettleMillis);
  traceAngles(label);
}

} // namespace

void setup() {
  // BSP の begin() が内部で M5.begin() を呼ぶ。ここで先に M5.begin() を
  // 済ませておくのは、BSP 初期化中の進捗を画面に出したいため。
  // M5Unified の begin() は二重に呼んでも安全。
  auto config = M5.config();
  M5.begin(config);

  M5.Display.setRotation(1);
  M5.Display.setTextSize(2);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

  // Why not Serial.begin(): CoreS3 は ARDUINO_USB_CDC_ON_BOOT=1 なので
  // Serial は起動時点で USB CDC として既に開いている。後から begin() を
  // 呼ぶと CDC の列挙が壊れ、ホストから一切データが見えなくなる (実機で確認)。

  trace("boot ok");
  trace("bsp begin...");

  // BSP の begin() は io_expander を最大 1200ms 待つ。ここで固まる可能性がある。
  M5StackChan.begin();
  trace("bsp begin done");

  // 既定では静止 200ms 後にトルクが切れて首が垂れる。指した姿勢を保つには必須。
  M5StackChan.Motion.setAutoTorqueReleaseEnabled(false);
  // 高頻度で角度を更新すると、同期が有効なままでは velocity がリセットされてカクつく。
  M5StackChan.Motion.setAutoAngleSyncEnabled(false);
  trace("motion cfg ok");

  traceAngles("start");

  // yaw を端から端まで。可動域いっぱいに動くかを目視と数値の両方で確認する。
  moveAndReport("yawMin", pointing::kYawMinDeci, pointing::kPitchLevelDeci);
  moveAndReport("yawMid", 0, pointing::kPitchLevelDeci);
  moveAndReport("yawMax", pointing::kYawMaxDeci, pointing::kPitchLevelDeci);
  moveAndReport("yawMid", 0, pointing::kPitchLevelDeci);

  // pitch の上端・下端。ここが本命の確認。
  moveAndReport("pitMin", 0, pointing::kPitchMinDeci);
  moveAndReport("pitLvl", 0, pointing::kPitchLevelDeci);
  moveAndReport("pitMax", 0, pointing::kPitchMaxDeci);

  const int reachedPitch = M5StackChan.Motion.getCurrentPitchAngle();
  saveResult("pitchMax", reachedPitch);

  // yaw の到達値も取る。両端に振ってから中央に戻し、端の値を覚えておく。
  M5StackChan.Motion.move(pointing::kYawMaxDeci, pointing::kPitchLevelDeci, kMoveSpeed);
  delay(kSettleMillis);
  const int reachedYawMax = M5StackChan.Motion.getCurrentYawAngle();
  saveResult("yawMax", reachedYawMax);

  M5StackChan.Motion.move(pointing::kYawMinDeci, pointing::kPitchLevelDeci, kMoveSpeed);
  delay(kSettleMillis);
  saveResult("yawMin", M5StackChan.Motion.getCurrentYawAngle());

  M5StackChan.Motion.move(0, pointing::kPitchLevelDeci, kMoveSpeed);
  delay(kSettleMillis);

  // トルク保持の確認: 指令を出さずに 5 秒放置し、角度がどれだけ動くかを測る。
  // setAutoTorqueReleaseEnabled(false) が効いていれば、ほぼ 0 のはず。
  const int holdStartPitch = M5StackChan.Motion.getCurrentPitchAngle();
  delay(5000);
  const int holdDrift = M5StackChan.Motion.getCurrentPitchAngle() - holdStartPitch;
  saveResult("holdDrift", holdDrift);

  // 結論は流さずに固定表示する。スクロールで消えると読めないため。
  const bool pitchReachesTop = reachedPitch >= pointing::kPitchMaxDeci - 50;
  auto& display = M5.Display;
  display.fillScreen(TFT_BLACK);
  display.setTextSize(2);
  display.setTextColor(pitchReachesTop ? TFT_GREEN : TFT_YELLOW, TFT_BLACK);
  display.setCursor(4, 4);
  display.printf("pitch max: %d", reachedPitch);
  display.setCursor(4, 30);
  display.printf("want>=%d %s", pointing::kPitchMaxDeci - 50, pitchReachesTop ? "OK" : "SHORT");
  display.setTextColor(TFT_CYAN, TFT_BLACK);
  display.setCursor(4, 60);
  display.printf("yaw max:   %d", reachedYawMax);
  display.setCursor(4, 86);
  display.printf("want>=%d", pointing::kYawMaxDeci - 50);
  display.setTextColor(holdDrift > -30 && holdDrift < 30 ? TFT_GREEN : TFT_RED, TFT_BLACK);
  display.setCursor(4, 116);
  display.printf("hold drift: %d", holdDrift);

  showPreviousResults();
}

void loop() {
  M5StackChan.update();

  // トルク保持が効いているか (首が垂れないか) を数値で確認する。
  // 画面下部を上書きし続けるので、値が動かなければ保持できている。
  static std::uint32_t lastReportMillis = 0;
  const std::uint32_t now = millis();
  if (now - lastReportMillis >= 1000) {
    lastReportMillis = now;
    auto& display = M5.Display;
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setCursor(4, 146);
    display.printf("now y=%5d p=%4d ", M5StackChan.Motion.getCurrentYawAngle(),
                   M5StackChan.Motion.getCurrentPitchAngle());
  }

  delay(10);
}
