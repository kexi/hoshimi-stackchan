// 段階 6: ファーム最小構成。
//
// ここで確かめたいのは 2 点だけ:
//   1. framework = arduino 単独で StackChan-BSP がビルド・動作するか
//   2. pitch が上端 (900) まで実際に届くか
//      (BSP は NVS の zero_pos_2 を原点に使い、raw を 0..1000 で clamp するので、
//       個体の zero_pos 次第では上端がサチる可能性がある)
//
// 磁気も天体計算もまだ触らない。順に足していく。

#include <M5StackChan.h>

#include "pointing/servo_map.hpp"

#include <cstdint>

namespace {

// 首を動かしてから、スプリング応答が収まるのを待つ時間。
// BSP の speed はバネ定数へのマッピングで、臨界減衰なのでオーバーシュートはしない。
constexpr std::uint32_t kSettleMillis = 1200;
constexpr int kMoveSpeed = 400;

void reportAngles(const char* label) {
  const int yaw = M5StackChan.Motion.getCurrentYawAngle();
  const int pitch = M5StackChan.Motion.getCurrentPitchAngle();
  Serial.printf("%-18s yaw=%5d (%6.1f deg)  pitch=%4d (%5.1f deg)\n", label, yaw, yaw / 10.0, pitch,
                pitch / 10.0);
}

void moveAndReport(const char* label, int yawDeci, int pitchDeci) {
  M5StackChan.Motion.move(yawDeci, pitchDeci, kMoveSpeed);
  delay(kSettleMillis);
  reportAngles(label);
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  // begin() は M5.begin() を内包する。別途 M5.begin() を呼ぶと二重初期化になる。
  M5StackChan.begin();

  // 既定では静止 200ms 後にトルクが切れて首が垂れる。指した姿勢を保つには必須。
  M5StackChan.Motion.setAutoTorqueReleaseEnabled(false);
  // 高頻度で角度を更新すると、同期が有効なままでは velocity がリセットされてカクつく。
  M5StackChan.Motion.setAutoAngleSyncEnabled(false);

  M5StackChan.Display().setRotation(1);
  M5StackChan.Display().setTextSize(2);
  M5StackChan.Display().println("compass-stackchan");
  M5StackChan.Display().println("stage 6: servo range");

  Serial.println();
  Serial.println("=== servo range check ===");
  Serial.printf("yaw   limits: %d .. %d (deci-degree)\n", pointing::kYawMinDeci,
                pointing::kYawMaxDeci);
  Serial.printf("pitch limits: %d .. %d (deci-degree), level=%d\n", pointing::kPitchMinDeci,
                pointing::kPitchMaxDeci, pointing::kPitchLevelDeci);

  reportAngles("boot");

  // yaw を端から端まで。可動域いっぱいに動くかを目視と数値の両方で確認する。
  moveAndReport("yaw min", pointing::kYawMinDeci, pointing::kPitchLevelDeci);
  moveAndReport("yaw center", 0, pointing::kPitchLevelDeci);
  moveAndReport("yaw max", pointing::kYawMaxDeci, pointing::kPitchLevelDeci);
  moveAndReport("yaw center", 0, pointing::kPitchLevelDeci);

  // pitch の上端・下端。ここが本命の確認。
  moveAndReport("pitch min", 0, pointing::kPitchMinDeci);
  moveAndReport("pitch level", 0, pointing::kPitchLevelDeci);
  moveAndReport("pitch max", 0, pointing::kPitchMaxDeci);

  const int reachedPitch = M5StackChan.Motion.getCurrentPitchAngle();
  const bool pitchReachesTop = reachedPitch >= pointing::kPitchMaxDeci - 50;
  Serial.printf("\npitch top reached: %s (got %d, want >= %d)\n", pitchReachesTop ? "YES" : "NO",
                reachedPitch, pointing::kPitchMaxDeci - 50);
  if (!pitchReachesTop) {
    // NVS の zero_pos_2 が上端側に寄っていると raw が 1000 で頭打ちになる。
    // その場合は setCurrentPostionAsHome() で原点を取り直す必要がある。
    Serial.println("  -> NVS zero_pos_2 の再調整が要るかもしれない");
  }

  moveAndReport("pitch level", 0, pointing::kPitchLevelDeci);
  Serial.println("=== done ===");
}

void loop() {
  M5StackChan.update();

  // トルク保持が効いているか (首が垂れないか) を目視で確認できるよう、
  // 10 秒ごとに現在角度を出し続ける。
  static std::uint32_t lastReportMillis = 0;
  const std::uint32_t now = millis();
  if (now - lastReportMillis >= 10000) {
    lastReportMillis = now;
    reportAngles("holding");
  }

  delay(10);
}
