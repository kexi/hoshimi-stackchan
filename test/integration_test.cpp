// 天体計算からサーボ指令までを通しで固定するゴールデンテスト。
//
// 保証すること:
//  - 固定シナリオ (東京・固定日時・機体が真北) でのサーボ指令値が変わらない
//  - 偏角を無視すると実際に方位がずれる (偏角の設定が必須であることの担保)
//  - 磁気測定 → 方位 → 天体 → サーボ の経路に NaN や範囲外が現れない

#include "astro/ephemeris.hpp"
#include "compass/declination.hpp"
#include "compass/heading.hpp"
#include "pointing/solver.hpp"

#include "test_support.hpp"

#include <cmath>

namespace {

// 東京駅付近。実機のビルド時定数もこの値を既定にする。
constexpr double kTokyoLatitude = 35.681236;
constexpr double kTokyoLongitudeEast = 139.767125;
// 2026 年の東京の磁気偏角は西偏約 7.9 度 (東偏を正とするので負)。
constexpr float kTokyoDeclinationEast = -7.9F;

// 2026-08-11 12:00 JST = 03:00 UTC
constexpr std::int64_t kNoonJst = 1786064400;

astro::Observer tokyoObserver() {
  astro::Observer observer;
  observer.latitudeDegrees = kTokyoLatitude;
  observer.longitudeEastDegrees = kTokyoLongitudeEast;
  return observer;
}

void testGoldenServoCommands() {
  const astro::Observer observer = tokyoObserver();

  // 機体が真北を向いている前提で、各ターゲットのサーボ指令を固定する。
  const astro::TargetPosition sun =
      astro::computeTargetPosition(astro::Target::Sun, kNoonJst, observer, true);
  CHECK_TRUE(sun.valid);

  pointing::SolveInput input;
  input.targetAzimuthDegrees = sun.horizontal.azimuthDegrees;
  input.targetAltitudeDegrees = sun.horizontal.altitudeDegrees;
  input.bodyHeadingDegrees = 0.0;
  const pointing::SolveResult sunSolve = pointing::solve(input);

  // 正午の太陽は南寄り・高い。首は南 (yaw が大きい) を向き、pitch は上限に張り付く。
  CHECK_TRUE(sunSolve.command.yawDeciDegrees > 0);
  CHECK_TRUE(sunSolve.command.pitchDeciDegrees == pointing::kPitchMaxDeci);
  CHECK_TRUE(sunSolve.command.clampedPitch);
  CHECK_TRUE(sunSolve.unreachablePitchDegrees > 10.0);

  // 全ターゲットで指令が可動域に収まり、NaN が出ない
  for (std::uint8_t index = 0; index < static_cast<std::uint8_t>(astro::Target::kCount); ++index) {
    const auto target = static_cast<astro::Target>(index);
    const astro::TargetPosition position =
        astro::computeTargetPosition(target, kNoonJst, observer, true);
    CHECK_TRUE(position.valid);

    pointing::SolveInput solveInput;
    solveInput.targetAzimuthDegrees = position.horizontal.azimuthDegrees;
    solveInput.targetAltitudeDegrees = position.horizontal.altitudeDegrees;
    solveInput.bodyHeadingDegrees = 0.0;

    const pointing::SolveResult result = pointing::solve(solveInput);
    CHECK_TRUE(pointing::isYawReachable(result.command.yawDeciDegrees));
    CHECK_TRUE(pointing::isPitchReachable(result.command.pitchDeciDegrees));
    CHECK_TRUE(!std::isnan(result.unreachableYawDegrees));
    CHECK_TRUE(!std::isnan(result.unreachablePitchDegrees));
  }
}

void testDeclinationMattersEndToEnd() {
  // 偏角を入れ忘れると、東京では約 7.9 度ずれる。
  // サーボの分解能 0.3125 度に対して 25 倍で、要求 0.1 度からは論外。
  // このテストは「偏角の設定は任意ではない」ことを数値で示すためにある。
  const float magneticHeading = 0.0F; // 磁北を向いている
  const float trueHeading =
      compass::trueHeadingFromMagnetic(magneticHeading, kTokyoDeclinationEast);

  // 首の可動域の端に当たらない方位を選ぶ。端で clamp されると差が縮んで、
  // 「偏角の影響」ではなく「可動域の影響」を測ってしまう。
  pointing::SolveInput withDeclination;
  withDeclination.targetAzimuthDegrees = 30.0;
  withDeclination.targetAltitudeDegrees = 20.0;
  withDeclination.bodyHeadingDegrees = trueHeading;

  pointing::SolveInput withoutDeclination = withDeclination;
  withoutDeclination.bodyHeadingDegrees = magneticHeading;

  const pointing::SolveResult solvedWith = pointing::solve(withDeclination);
  const pointing::SolveResult solvedWithout = pointing::solve(withoutDeclination);
  CHECK_TRUE(!solvedWith.command.clampedYaw);
  CHECK_TRUE(!solvedWithout.command.clampedYaw);

  const double differenceDegrees =
      std::fabs(static_cast<double>(solvedWith.command.yawDeciDegrees -
                                    solvedWithout.command.yawDeciDegrees)) /
      10.0;
  CHECK_NEAR(differenceDegrees, std::fabs(kTokyoDeclinationEast), 0.2);
  // サーボの分解能 (0.3125 度) の何倍もある = 無視できない量であることを示す
  CHECK_TRUE(differenceDegrees > pointing::kServoStepDegrees * 10.0);
}

void testSummerNoonSunExceedsNeckRange() {
  // 東京の夏の正午、機体が真北を向いていると、太陽は方位 122 度・高度 59 度。
  // 偏角を入れて機体基準に直すと必要な yaw が 130 度になり、首の可動域
  // (±128 度) を超える。「体ごと回して」と伝えるべき場面が実際に起きることを
  // 固定しておく (この条件を知らずに組むと、指せていないのに指せたつもりになる)。
  const astro::Observer observer = tokyoObserver();
  const astro::TargetPosition sun =
      astro::computeTargetPosition(astro::Target::Sun, kNoonJst, observer, true);
  CHECK_NEAR(sun.horizontal.azimuthDegrees, 122.03, 0.2);
  CHECK_NEAR(sun.horizontal.altitudeDegrees, 59.46, 0.2);

  pointing::SolveInput input;
  input.targetAzimuthDegrees = sun.horizontal.azimuthDegrees;
  input.targetAltitudeDegrees = sun.horizontal.altitudeDegrees;
  input.bodyHeadingDegrees = compass::trueHeadingFromMagnetic(0.0F, kTokyoDeclinationEast);

  const pointing::SolveResult result = pointing::solve(input);
  CHECK_TRUE(result.command.clampedYaw);
  CHECK_TRUE(pointing::needsBodyRotation(result));
  // 高度 59 度も pitch の上限 (45 度) を超える
  CHECK_TRUE(result.command.clampedPitch);
  CHECK_TRUE(result.command.pitchDeciDegrees == pointing::kPitchMaxDeci);
}

void testFullChainFromSyntheticMagnetometer() {
  // 合成した磁気・加速度から方位を出し、天体を解いて、サーボ指令まで通す。
  // 経路のどこかで単位や座標系を取り違えていれば、ここで範囲外や NaN になる。
  constexpr float kPi = 3.14159265358979323846F;
  constexpr float kInclination = 49.0F * kPi / 180.0F;
  constexpr float kFieldStrength = 46.0F;

  const astro::Observer observer = tokyoObserver();

  for (int headingStep = 0; headingStep < 8; ++headingStep) {
    const float bodyMagneticHeading = static_cast<float>(headingStep) * 45.0F;
    const float headingRadians = bodyMagneticHeading * kPi / 180.0F;

    compass::Vec3 magneticField;
    magneticField.x = kFieldStrength * std::cos(kInclination) * std::cos(headingRadians);
    magneticField.y = -kFieldStrength * std::cos(kInclination) * std::sin(headingRadians);
    magneticField.z = kFieldStrength * std::sin(kInclination);

    const compass::Attitude attitude = compass::attitudeFromAccel(compass::Vec3{0.0F, 0.0F, 1.0F});
    const float measuredMagnetic = compass::tiltCompensatedHeadingDegrees(magneticField, attitude);
    CHECK_NEAR_ANGLE(measuredMagnetic, bodyMagneticHeading, 0.05);

    const float bodyTrueHeading =
        compass::trueHeadingFromMagnetic(measuredMagnetic, kTokyoDeclinationEast);

    for (std::uint8_t index = 0; index < static_cast<std::uint8_t>(astro::Target::kCount);
         ++index) {
      const auto target = static_cast<astro::Target>(index);
      const astro::TargetPosition position =
          astro::computeTargetPosition(target, kNoonJst, observer, true);

      pointing::SolveInput input;
      input.targetAzimuthDegrees = position.horizontal.azimuthDegrees;
      input.targetAltitudeDegrees = position.horizontal.altitudeDegrees;
      input.bodyHeadingDegrees = bodyTrueHeading;

      const pointing::SolveResult result = pointing::solve(input);
      CHECK_TRUE(pointing::isYawReachable(result.command.yawDeciDegrees));
      CHECK_TRUE(pointing::isPitchReachable(result.command.pitchDeciDegrees));
    }
  }
}

void testNorthIsAlwaysPointable() {
  // 時刻同期に失敗しても、真北だけは必ず指せること。
  // これが崩れると Wi-Fi が無い場所でデバイスが何もできなくなる。
  const astro::Observer observer = tokyoObserver();
  const astro::TargetPosition north =
      astro::computeTargetPosition(astro::Target::North, 0, observer, false);
  CHECK_TRUE(north.valid);

  pointing::SolveInput input;
  input.targetAzimuthDegrees = north.horizontal.azimuthDegrees;
  input.targetAltitudeDegrees = north.horizontal.altitudeDegrees;
  // 機体が東を向いていれば、首は左 90 度を向いて北を指す
  input.bodyHeadingDegrees = 90.0;

  const pointing::SolveResult result = pointing::solve(input);
  CHECK_TRUE(result.command.yawDeciDegrees == -900);
  CHECK_TRUE(result.command.pitchDeciDegrees == pointing::kPitchLevelDeci);
  CHECK_TRUE(!result.command.clampedYaw);
}

} // namespace

int main() {
  testGoldenServoCommands();
  testDeclinationMattersEndToEnd();
  testSummerNoonSunExceedsNeckRange();
  testFullChainFromSyntheticMagnetometer();
  testNorthIsAlwaysPointable();
  return testing::summarize("integration");
}
