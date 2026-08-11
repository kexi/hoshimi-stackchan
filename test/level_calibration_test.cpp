// level_calibration が保証すること:
//  - 本体を水平に一回転させた点から、水平面の円を復元する
//  - 実機規模のハードアイアン (地磁気の 6 倍) があっても方位が出る
//  - 一回転していない (半周など) データは採用しない
//  - 補正後の方位が、機体の実際の向きと一致する

#include "compass/heading.hpp"
#include "compass/level_calibration.hpp"

#include "test_support.hpp"

#include <cmath>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846F;
// 日本の地磁気。水平成分は約 30uT、伏角 49 度。
constexpr float kHorizontalField = 30.0F;
constexpr float kVerticalField = 34.0F;
// 実機で観測されたハードアイアン。地磁気の 6 倍ある。
constexpr float kHardIronX = -95.0F;
constexpr float kHardIronY = 145.0F;
constexpr float kHardIronZ = 340.0F;

// 本体を水平に回したときに観測される磁気。
// K151実機の顔座標では、時計回りの方位増加に対して水平Yが正へ回る。
compass::Vec3 sampleAtHeading(float headingDegrees, float scaleX, float scaleY) {
  const float radians = headingDegrees * kPi / 180.0F;
  compass::Vec3 sample;
  sample.x = kHardIronX + kHorizontalField * std::cos(radians) / scaleX;
  sample.y = kHardIronY + kHorizontalField * std::sin(radians) / scaleY;
  sample.z = kHardIronZ + kVerticalField;
  return sample;
}

std::vector<compass::Vec3> fullTurn(int count, float scaleX = 1.0F, float scaleY = 1.0F) {
  std::vector<compass::Vec3> points;
  points.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    const float heading = static_cast<float>(index) * 360.0F / static_cast<float>(count);
    points.push_back(sampleAtHeading(heading, scaleX, scaleY));
  }
  return points;
}

compass::Vec3 tiltedRawSample(float headingDegrees, compass::Attitude attitude) {
  const float heading = headingDegrees * kPi / 180.0F;
  const float sinRoll = std::sin(attitude.rollRadians);
  const float cosRoll = std::cos(attitude.rollRadians);
  const float sinPitch = std::sin(attitude.pitchRadians);
  const float cosPitch = std::cos(attitude.pitchRadians);

  // heading.cppの水平化行列の転置で、水平座標の地磁気を機体座標へ戻す。
  const float horizontalX = kHorizontalField * std::cos(heading);
  const float horizontalY = kHorizontalField * std::sin(heading);
  const float vertical = kVerticalField;

  compass::Vec3 raw;
  raw.x = cosPitch * horizontalX - sinPitch * vertical + kHardIronX;
  raw.y = sinRoll * sinPitch * horizontalX + cosRoll * horizontalY + sinRoll * cosPitch * vertical +
          kHardIronY;
  raw.z = cosRoll * sinPitch * horizontalX - sinRoll * horizontalY + cosRoll * cosPitch * vertical +
          kHardIronZ;
  return raw;
}

compass::Vec3 rotatedEllipseSample(float headingDegrees) {
  constexpr float kMajorCorrection = 1.3F;
  constexpr float kMinorCorrection = 0.75F;
  constexpr float kAxisAngle = 32.0F * kPi / 180.0F;
  const float cosine = std::cos(kAxisAngle);
  const float sine = std::sin(kAxisAngle);
  const float inverseXX = cosine * cosine / kMajorCorrection + sine * sine / kMinorCorrection;
  const float inverseXY = cosine * sine * (1.0F / kMajorCorrection - 1.0F / kMinorCorrection);
  const float inverseYY = sine * sine / kMajorCorrection + cosine * cosine / kMinorCorrection;
  const float heading = headingDegrees * kPi / 180.0F;
  const float idealX = kHorizontalField * std::cos(heading);
  const float idealY = kHorizontalField * std::sin(heading);
  return compass::Vec3{kHardIronX + inverseXX * idealX + inverseXY * idealY,
                       kHardIronY + inverseXY * idealX + inverseYY * idealY, kHardIronZ};
}

void testRecoversHardIronFromLevelTurn() {
  // 一回転すれば、地磁気の 6 倍のハードアイアンがあっても中心が求まる。
  // これが 8 の字回しとの違い。本体を傾けないので磁石との相対関係が保たれる。
  const std::vector<compass::Vec3> points = fullTurn(72);
  const compass::LevelCalibration calibration =
      compass::fitLevelCircle(points.data(), points.size());

  CHECK_TRUE(calibration.valid);
  CHECK_NEAR(calibration.offsetX, kHardIronX, 0.5);
  CHECK_NEAR(calibration.offsetY, kHardIronY, 0.5);
  CHECK_NEAR(calibration.radius, kHorizontalField, 0.5);
  CHECK_TRUE(calibration.normalizedResidual < 0.02F);
}

void testCorrectedHeadingMatchesActual() {
  // 本命。補正後の方位が、機体が実際に向いている方位と一致すること。
  const std::vector<compass::Vec3> points = fullTurn(72);
  const compass::LevelCalibration calibration =
      compass::fitLevelCircle(points.data(), points.size());
  CHECK_TRUE(calibration.valid);

  const compass::Attitude level; // 水平に置いている
  for (int step = 0; step < 12; ++step) {
    const float heading = static_cast<float>(step) * 30.0F;
    const compass::Vec3 raw = sampleAtHeading(heading, 1.0F, 1.0F);
    const compass::Vec3 corrected = compass::applyLevelCalibration(calibration, raw);
    CHECK_NEAR_ANGLE(compass::tiltCompensatedHeadingDegrees(corrected, level), heading, 0.5);
  }
}

void testCorrectsAxisSensitivityDifference() {
  // X と Y の感度が違って円が楕円になっていても、方位が出ること。
  constexpr float kScaleX = 1.3F;
  constexpr float kScaleY = 0.8F;
  const std::vector<compass::Vec3> points = fullTurn(72, kScaleX, kScaleY);
  const compass::LevelCalibration calibration =
      compass::fitLevelCircle(points.data(), points.size());
  CHECK_TRUE(calibration.valid);

  const compass::Attitude level;
  for (int step = 0; step < 12; ++step) {
    const float heading = static_cast<float>(step) * 30.0F;
    const compass::Vec3 raw = sampleAtHeading(heading, kScaleX, kScaleY);
    const compass::Vec3 corrected = compass::applyLevelCalibration(calibration, raw);
    // 楕円を円に戻すので、補正しなければ数度ずれるところが 1 度以内に収まる
    CHECK_NEAR_ANGLE(compass::tiltCompensatedHeadingDegrees(corrected, level), heading, 1.0);
  }
}

void testCorrectsRotatedAxisSensitivityDifference() {
  // 軟鉄歪みの主軸がセンサーX/Yと一致しなくても、交差項を含めて補正すること。
  std::vector<compass::Vec3> points;
  points.reserve(72);
  for (int step = 0; step < 72; ++step) {
    points.push_back(rotatedEllipseSample(static_cast<float>(step) * 5.0F));
  }
  const compass::LevelCalibration calibration =
      compass::fitLevelCircle(points.data(), points.size());
  CHECK_TRUE(calibration.valid);
  CHECK_TRUE(std::fabs(calibration.crossAxis) > 0.05F);

  for (int step = 0; step < 12; ++step) {
    const float heading = static_cast<float>(step) * 30.0F;
    const compass::Vec3 corrected =
        compass::applyLevelCalibration(calibration, rotatedEllipseSample(heading));
    CHECK_NEAR_ANGLE(compass::headingDegreesFromHorizontal(corrected), heading, 1.0);
  }
}

void testTiltedProjectionCorrectsThreeAxisHardIron() {
  // CoreS3が傾いていてZにも強い固定磁場がある実機条件でも、先に水平投影すれば
  // 固定磁場は円の中心となり、全方位を復元できること。
  compass::Attitude attitude;
  attitude.rollRadians = 18.0F * kPi / 180.0F;
  attitude.pitchRadians = -37.0F * kPi / 180.0F;

  std::vector<compass::Vec3> horizontalSamples;
  horizontalSamples.reserve(72);
  for (int step = 0; step < 72; ++step) {
    const float heading = static_cast<float>(step) * 5.0F;
    const compass::Vec3 raw = tiltedRawSample(heading, attitude);
    horizontalSamples.push_back(compass::horizontalMagneticComponents(raw, attitude));
  }

  const compass::LevelCalibration calibration =
      compass::fitLevelCircle(horizontalSamples.data(), horizontalSamples.size());
  CHECK_TRUE(calibration.valid);

  for (int step = 0; step < 12; ++step) {
    const float heading = static_cast<float>(step) * 30.0F;
    const compass::Vec3 raw = tiltedRawSample(heading, attitude);
    const compass::Vec3 horizontal = compass::horizontalMagneticComponents(raw, attitude);
    const compass::Vec3 corrected = compass::applyLevelCalibration(calibration, horizontal);
    CHECK_NEAR_ANGLE(compass::headingDegreesFromHorizontal(corrected), heading, 0.5);
  }
}

void testRejectsPartialTurn() {
  // 半周しか回していないと中心がずれる。採用してはいけない。
  std::vector<compass::Vec3> halfTurn;
  halfTurn.reserve(40);
  for (int index = 0; index < 40; ++index) {
    halfTurn.push_back(sampleAtHeading(static_cast<float>(index) * 180.0F / 40.0F, 1.0F, 1.0F));
  }
  CHECK_TRUE(!compass::fitLevelCircle(halfTurn.data(), halfTurn.size()).valid);

  // 四半分ならなおさら
  std::vector<compass::Vec3> quarterTurn;
  quarterTurn.reserve(30);
  for (int index = 0; index < 30; ++index) {
    quarterTurn.push_back(sampleAtHeading(static_cast<float>(index) * 90.0F / 30.0F, 1.0F, 1.0F));
  }
  CHECK_TRUE(!compass::fitLevelCircle(quarterTurn.data(), quarterTurn.size()).valid);
}

void testCoverageReflectsRotation() {
  // 進捗表示に使うので、回した量に応じて増えること。
  const std::vector<compass::Vec3> full = fullTurn(72);
  CHECK_TRUE(compass::angularCoverage(full.data(), full.size(), kHardIronX, kHardIronY) > 0.95F);

  std::vector<compass::Vec3> half;
  half.reserve(36);
  for (int index = 0; index < 36; ++index) {
    half.push_back(sampleAtHeading(static_cast<float>(index) * 180.0F / 36.0F, 1.0F, 1.0F));
  }
  const float halfCoverage =
      compass::angularCoverage(half.data(), half.size(), kHardIronX, kHardIronY);
  CHECK_TRUE(halfCoverage > 0.4F);
  CHECK_TRUE(halfCoverage < 0.6F);
}

void testRejectsDegenerateInput() {
  CHECK_TRUE(!compass::fitLevelCircle(nullptr, 100).valid);

  const compass::Vec3 few[4] = {
      {0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {1.0F, 1.0F, 0.0F}};
  CHECK_TRUE(!compass::fitLevelCircle(few, 4).valid);

  // 一点に固まっている (機体が動いていない)
  std::vector<compass::Vec3> stationary;
  stationary.reserve(50);
  for (int index = 0; index < 50; ++index) {
    stationary.push_back(compass::Vec3{kHardIronX, kHardIronY, kHardIronZ});
  }
  CHECK_TRUE(!compass::fitLevelCircle(stationary.data(), stationary.size()).valid);
}

void testToleratesSensorNoise() {
  // 実機のノイズ (実測 1.9 度相当) を載せても方位が出ること。
  std::vector<compass::Vec3> noisy;
  noisy.reserve(144);
  for (int index = 0; index < 144; ++index) {
    const float heading = static_cast<float>(index) * 360.0F / 144.0F;
    compass::Vec3 point = sampleAtHeading(heading, 1.0F, 1.0F);
    // 決定的な擬似ノイズ。乱数は使わない (テストを再現可能に保つため)
    const float noise = std::sin(static_cast<float>(index) * 2.4F) * 0.8F;
    point.x += noise;
    point.y += std::cos(static_cast<float>(index) * 1.7F) * 0.8F;
    noisy.push_back(point);
  }

  const compass::LevelCalibration calibration = compass::fitLevelCircle(noisy.data(), noisy.size());
  CHECK_TRUE(calibration.valid);
  CHECK_NEAR(calibration.offsetX, kHardIronX, 1.5);
  CHECK_NEAR(calibration.offsetY, kHardIronY, 1.5);

  const compass::Attitude level;
  const compass::Vec3 raw = sampleAtHeading(0.0F, 1.0F, 1.0F);
  const compass::Vec3 corrected = compass::applyLevelCalibration(calibration, raw);
  CHECK_NEAR_ANGLE(compass::tiltCompensatedHeadingDegrees(corrected, level), 0.0, 3.0);
}

} // namespace

int main() {
  testRecoversHardIronFromLevelTurn();
  testCorrectedHeadingMatchesActual();
  testCorrectsAxisSensitivityDifference();
  testCorrectsRotatedAxisSensitivityDifference();
  testTiltedProjectionCorrectsThreeAxisHardIron();
  testRejectsPartialTurn();
  testCoverageReflectsRotation();
  testRejectsDegenerateInput();
  testToleratesSensorNoise();
  return testing::summarize("level-calibration");
}
