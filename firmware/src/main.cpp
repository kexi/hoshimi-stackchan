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

#include "pointing/servo_map.hpp"

#include <cstdint>
#include <cstdio>

namespace {

// 首を動かしてから、スプリング応答が収まるのを待つ時間。
// BSP の speed はバネ定数へのマッピングで、臨界減衰なのでオーバーシュートはしない。
constexpr std::uint32_t kSettleMillis = 1200;
constexpr int kMoveSpeed = 400;

int g_lineY = 0;

// LCD とシリアルの両方に出す。どちらか一方しか見えない状況でも追える。
void trace(const char* message) {
  Serial.println(message);
  Serial.flush();

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
  // M5StackChan.begin() は内部で M5.begin() を呼ぶが、そこまで到達せずに
  // 落ちている可能性を切り分けたいので、先に表示だけ自前で起こす。
  // 二重初期化にならないよう、ここでは M5.begin() は呼ばない。
  auto config = M5.config();
  M5.begin(config);

  M5.Display.setRotation(1);
  M5.Display.setTextSize(2);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);

  Serial.begin(115200);

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
  const bool pitchReachesTop = reachedPitch >= pointing::kPitchMaxDeci - 50;
  char verdict[96];
  std::snprintf(verdict, sizeof(verdict), "pitch top: %s (%d)", pitchReachesTop ? "YES" : "NO",
                reachedPitch);
  trace(verdict);
  if (!pitchReachesTop) {
    // NVS の zero_pos_2 が上端側に寄っていると raw が 1000 で頭打ちになる。
    // その場合は setCurrentPostionAsHome() で原点を取り直す必要がある。
    trace("check NVS zero_pos_2");
  }

  moveAndReport("pitLvl", 0, pointing::kPitchLevelDeci);
  trace("=== done ===");
}

void loop() {
  M5StackChan.update();

  // トルク保持が効いているか (首が垂れないか) を目視で確認できるよう、
  // 10 秒ごとに現在角度を出し続ける。
  static std::uint32_t lastReportMillis = 0;
  const std::uint32_t now = millis();
  if (now - lastReportMillis >= 10000) {
    lastReportMillis = now;
    traceAngles("hold");
  }

  delay(10);
}
