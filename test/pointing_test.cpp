// pointing_core が保証すること:
//  - 高度 → pitch の対応が BSP の可動域 (0..900, 450 が水平) と一致し、往復で戻る
//  - 可動域を超える高度・方位で clamp され、届かなかった量が残差として出る
//  - deadband が小さな変化を吸収する
//  - 地平線下ポリシーの 2 分岐がそれぞれ意図通りに効く

#include "pointing/servo_map.hpp"
#include "pointing/solver.hpp"

#include "test_support.hpp"

#include <cmath>

namespace {

void testPitchMapping() {
  // 水平が中央、±45 度で可動域の端
  CHECK_TRUE(pointing::pitchDeciFromAltitude(0.0) == pointing::kPitchLevelDeci);
  CHECK_TRUE(pointing::pitchDeciFromAltitude(45.0) == pointing::kPitchMaxDeci);
  CHECK_TRUE(pointing::pitchDeciFromAltitude(-45.0) == pointing::kPitchMinDeci);
  CHECK_TRUE(pointing::pitchDeciFromAltitude(10.0) == 550);
  CHECK_TRUE(pointing::pitchDeciFromAltitude(-10.0) == 350);

  // 可動域外は端に貼りつく
  CHECK_TRUE(pointing::pitchDeciFromAltitude(90.0) == pointing::kPitchMaxDeci);
  CHECK_TRUE(pointing::pitchDeciFromAltitude(-90.0) == pointing::kPitchMinDeci);

  // 表現できる範囲では往復で戻る。
  // ループ変数は整数で回す (double を加算し続けると誤差が溜まり、
  // 境界値をまたぐかどうかが処理系依存になる)。
  for (int step = -30; step <= 30; ++step) {
    const double altitude = static_cast<double>(step) * 1.5;
    const int pitchDeci = pointing::pitchDeciFromAltitude(altitude);
    CHECK_NEAR(pointing::altitudeFromPitchDeci(pitchDeci), altitude, 0.06);
    CHECK_TRUE(pointing::isPitchReachable(pitchDeci));
  }
}

void testYawMapping() {
  CHECK_TRUE(pointing::yawDeciFromRelativeBearing(0.0) == 0);
  CHECK_TRUE(pointing::yawDeciFromRelativeBearing(90.0) == 900);
  CHECK_TRUE(pointing::yawDeciFromRelativeBearing(-90.0) == -900);
  CHECK_TRUE(pointing::yawDeciFromRelativeBearing(128.0) == pointing::kYawMaxDeci);

  // 可動域 (±128 度) を超える方位は端に貼りつく
  CHECK_TRUE(pointing::yawDeciFromRelativeBearing(170.0) == pointing::kYawMaxDeci);
  CHECK_TRUE(pointing::yawDeciFromRelativeBearing(-170.0) == pointing::kYawMinDeci);

  // 360 度をまたいでも最短側に畳まれる
  CHECK_TRUE(pointing::yawDeciFromRelativeBearing(370.0) == 100);
  CHECK_TRUE(pointing::yawDeciFromRelativeBearing(-350.0) == 100);
}

void testServoArrivalRequiresBothAxes() {
  // yaw と pitch の両方が許容差内に入ったときだけ、到達済みと判定すること。
  constexpr int kToleranceDeci = 100;
  CHECK_TRUE(pointing::isServoAtTarget(905, 705, 900, 700, kToleranceDeci));
  CHECK_TRUE(!pointing::isServoAtTarget(1100, 705, 900, 700, kToleranceDeci));
  CHECK_TRUE(!pointing::isServoAtTarget(905, 900, 900, 700, kToleranceDeci));
}

void testSolveBasic() {
  // 機体が真北を向いていて、目標が真東・高度 30 度
  pointing::SolveInput input;
  input.targetAzimuthDegrees = 90.0;
  input.targetAltitudeDegrees = 30.0;
  input.bodyHeadingDegrees = 0.0;
  input.currentYawDeciDegrees = 0;
  input.currentPitchDeciDegrees = pointing::kPitchLevelDeci;

  const pointing::SolveResult result = pointing::solve(input);
  CHECK_TRUE(result.command.yawDeciDegrees == 900);
  CHECK_TRUE(result.command.pitchDeciDegrees == 750);
  CHECK_TRUE(result.shouldMove);
  CHECK_TRUE(!result.belowHorizon);
  CHECK_TRUE(!result.command.clampedYaw);
  CHECK_TRUE(!result.command.clampedPitch);
  CHECK_TRUE(!pointing::needsBodyRotation(result));

  // 機体が既に東を向いていれば、首は正面のまま
  input.bodyHeadingDegrees = 90.0;
  const pointing::SolveResult facingTarget = pointing::solve(input);
  CHECK_TRUE(facingTarget.command.yawDeciDegrees == 0);
}

void testSolveClampsHighAltitude() {
  // 日本の夏の南中太陽 (高度 72 度) は pitch の上限を超える。
  // 「指せていない」ことが残差として出ることが重要。
  pointing::SolveInput input;
  input.targetAzimuthDegrees = 180.0;
  input.targetAltitudeDegrees = 72.0;
  input.bodyHeadingDegrees = 180.0;

  const pointing::SolveResult result = pointing::solve(input);
  CHECK_TRUE(result.command.pitchDeciDegrees == pointing::kPitchMaxDeci);
  CHECK_TRUE(result.command.clampedPitch);
  CHECK_NEAR(result.unreachablePitchDegrees, 27.0, 0.1);
  CHECK_TRUE(!result.command.clampedYaw);
}

void testSolveClampsWideBearing() {
  // 真後ろ (相対方位 170 度) は首の可動域 ±128 度を超える
  pointing::SolveInput input;
  input.targetAzimuthDegrees = 170.0;
  input.targetAltitudeDegrees = 10.0;
  input.bodyHeadingDegrees = 0.0;

  const pointing::SolveResult result = pointing::solve(input);
  CHECK_TRUE(result.command.yawDeciDegrees == pointing::kYawMaxDeci);
  CHECK_TRUE(result.command.clampedYaw);
  CHECK_NEAR(result.unreachableYawDegrees, 42.0, 0.1);
  // 体ごと回してもらう必要がある
  CHECK_TRUE(pointing::needsBodyRotation(result));
}

void testDeadband() {
  pointing::SolveInput input;
  input.targetAzimuthDegrees = 0.0;
  input.targetAltitudeDegrees = 0.0;
  input.bodyHeadingDegrees = 0.0;
  input.currentYawDeciDegrees = 0;
  input.currentPitchDeciDegrees = pointing::kPitchLevelDeci;
  input.deadbandDegrees = 0.5;

  // 0.3 度の変化では動かない
  input.targetAzimuthDegrees = 0.3;
  CHECK_TRUE(!pointing::solve(input).shouldMove);

  // 0.8 度なら動く
  input.targetAzimuthDegrees = 0.8;
  CHECK_TRUE(pointing::solve(input).shouldMove);

  // pitch だけが動く場合も拾う
  input.targetAzimuthDegrees = 0.0;
  input.targetAltitudeDegrees = 1.0;
  CHECK_TRUE(pointing::solve(input).shouldMove);
}

void testBelowHorizonPolicies() {
  pointing::SolveInput input;
  input.targetAzimuthDegrees = 45.0;
  input.targetAltitudeDegrees = -20.0;
  input.bodyHeadingDegrees = 0.0;

  // PointAnyway: 方位はそのまま、pitch は下を向く
  input.belowHorizonPolicy = pointing::BelowHorizonPolicy::PointAnyway;
  const pointing::SolveResult pointing_ = pointing::solve(input);
  CHECK_TRUE(pointing_.belowHorizon);
  CHECK_TRUE(pointing_.command.yawDeciDegrees == 450);
  CHECK_TRUE(pointing_.command.pitchDeciDegrees == 250);

  // ClampToLevel: 方位はそのまま、pitch は水平で止まる
  input.belowHorizonPolicy = pointing::BelowHorizonPolicy::ClampToLevel;
  const pointing::SolveResult clamped = pointing::solve(input);
  CHECK_TRUE(clamped.belowHorizon);
  CHECK_TRUE(clamped.command.yawDeciDegrees == 450);
  CHECK_TRUE(clamped.command.pitchDeciDegrees == pointing::kPitchLevelDeci);

  // 深く沈んだ天体 (-70 度) は下限に貼りつき、残差が出る
  input.targetAltitudeDegrees = -70.0;
  input.belowHorizonPolicy = pointing::BelowHorizonPolicy::PointAnyway;
  const pointing::SolveResult deep = pointing::solve(input);
  CHECK_TRUE(deep.command.pitchDeciDegrees == pointing::kPitchMinDeci);
  CHECK_TRUE(deep.command.clampedPitch);
  CHECK_NEAR(deep.unreachablePitchDegrees, -25.0, 0.1);
}

void testCommandsStayInRange() {
  // どんな入力でも、発行するコマンドは必ず BSP の可動域に収まっていること。
  // ここを守れないと BSP 側で無言で clamp され、指している方向と
  // 内部状態がずれる。
  for (int azimuthStep = 0; azimuthStep < 33; ++azimuthStep) {
    for (int altitudeStep = 0; altitudeStep <= 26; ++altitudeStep) {
      for (int headingStep = 0; headingStep < 7; ++headingStep) {
        const double azimuth = static_cast<double>(azimuthStep) * 11.0;
        const double altitude = -90.0 + static_cast<double>(altitudeStep) * 7.0;
        const double bodyHeading = static_cast<double>(headingStep) * 53.0;

        pointing::SolveInput input;
        input.targetAzimuthDegrees = azimuth;
        input.targetAltitudeDegrees = altitude;
        input.bodyHeadingDegrees = bodyHeading;

        const pointing::SolveResult result = pointing::solve(input);
        CHECK_TRUE(pointing::isYawReachable(result.command.yawDeciDegrees));
        CHECK_TRUE(pointing::isPitchReachable(result.command.pitchDeciDegrees));
      }
    }
  }
}

} // namespace

int main() {
  testPitchMapping();
  testYawMapping();
  testServoArrivalRequiresBothAxes();
  testSolveBasic();
  testSolveClampsHighAltitude();
  testSolveClampsWideBearing();
  testDeadband();
  testBelowHorizonPolicies();
  testCommandsStayInRange();
  return testing::summarize("pointing");
}
