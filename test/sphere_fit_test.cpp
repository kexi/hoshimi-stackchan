// sphere_fit が保証すること:
//  - 既知の中心・半径の球を、部分的にしか掃いていなくても復元する
//  - min/max 方式が失敗する条件で成功する (これが導入の理由)
//  - 球に乗っていないデータでは残差が大きく出る

#include "compass/calibration.hpp"
#include "compass/sphere_fit.hpp"

#include "test_support.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846F;

// 球面上の点を作る。azimuthTurns / elevationSpan で掃く範囲を変えられる。
std::vector<compass::Vec3> sphereSamples(compass::Vec3 center, float radius, float azimuthTurns,
                                         float elevationSpanRadians, int count) {
  std::vector<compass::Vec3> points;
  points.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    const float t = static_cast<float>(index) / static_cast<float>(count);
    const float azimuth = t * azimuthTurns * 2.0F * kPi;
    const float elevation = (t - 0.5F) * elevationSpanRadians;
    compass::Vec3 point;
    point.x = center.x + radius * std::cos(elevation) * std::cos(azimuth);
    point.y = center.y + radius * std::cos(elevation) * std::sin(azimuth);
    point.z = center.z + radius * std::sin(elevation);
    points.push_back(point);
  }
  return points;
}

void testRecoversKnownSphere() {
  const compass::Vec3 center{-125.0F, 14.0F, 340.0F}; // 実機で観測された規模
  constexpr float kRadius = 46.0F;

  const std::vector<compass::Vec3> points = sphereSamples(center, kRadius, 3.0F, kPi * 0.9F, 400);
  const compass::SphereFit fit = compass::fitSphere(points.data(), points.size());

  CHECK_TRUE(fit.valid);
  CHECK_NEAR(fit.center.x, center.x, 0.5);
  CHECK_NEAR(fit.center.y, center.y, 0.5);
  CHECK_NEAR(fit.center.z, center.z, 0.5);
  CHECK_NEAR(fit.radius, kRadius, 0.5);
  // 完全に球面上の点なので残差はほぼゼロ
  CHECK_TRUE(fit.normalizedResidual < 0.01F);
}

void testSucceedsWherePeakToPeakFails() {
  // これが sphere_fit を入れた理由。
  //
  // 水平方向に半周しか回していない状況を作る。min/max はその範囲の端しか
  // 見ないので中心が大きくずれるが、最小二乗なら全点が中心の推定に効く。
  const compass::Vec3 center{-125.0F, 14.0F, 340.0F};
  constexpr float kRadius = 46.0F;

  std::vector<compass::Vec3> points;
  for (int index = 0; index < 200; ++index) {
    const float t = static_cast<float>(index) / 200.0F;
    // 方位は半周 (180 度) だけ、仰角も浅い
    const float azimuth = t * kPi;
    const float elevation = (t - 0.5F) * (kPi / 3.0F);
    compass::Vec3 point;
    point.x = center.x + kRadius * std::cos(elevation) * std::cos(azimuth);
    point.y = center.y + kRadius * std::cos(elevation) * std::sin(azimuth);
    point.z = center.z + kRadius * std::sin(elevation);
    points.push_back(point);
  }

  // min/max 方式で中心を出すと、掃いた範囲の中心に寄ってずれる
  compass::Vec3 minValue = points[0];
  compass::Vec3 maxValue = points[0];
  for (const compass::Vec3& point : points) {
    minValue.x = std::fmin(minValue.x, point.x);
    minValue.y = std::fmin(minValue.y, point.y);
    minValue.z = std::fmin(minValue.z, point.z);
    maxValue.x = std::fmax(maxValue.x, point.x);
    maxValue.y = std::fmax(maxValue.y, point.y);
    maxValue.z = std::fmax(maxValue.z, point.z);
  }
  const float peakToPeakCenterY = 0.5F * (minValue.y + maxValue.y);
  const float peakToPeakErrorY = std::fabs(peakToPeakCenterY - center.y);

  // 最小二乗なら正しく求まる
  const compass::SphereFit fit = compass::fitSphere(points.data(), points.size());
  CHECK_TRUE(fit.valid);
  const float fitErrorY = std::fabs(fit.center.y - center.y);

  CHECK_NEAR(fit.center.x, center.x, 1.0);
  CHECK_NEAR(fit.center.y, center.y, 1.0);
  CHECK_NEAR(fit.radius, kRadius, 1.0);
  // min/max より明確に良いこと。これが言えないと導入の意味がない。
  CHECK_TRUE(fitErrorY < peakToPeakErrorY);
  CHECK_TRUE(peakToPeakErrorY > 5.0F); // min/max は実際に大きく外している
}

void testCalibrationFromSphere() {
  const compass::Vec3 center{-125.0F, 14.0F, 340.0F};
  constexpr float kRadius = 46.0F;
  const std::vector<compass::Vec3> points = sphereSamples(center, kRadius, 3.0F, kPi * 0.9F, 400);

  const compass::SphereFit fit = compass::fitSphere(points.data(), points.size());
  const compass::MagCalibration calibration =
      compass::calibrationFromSphere(fit, points.data(), points.size());

  CHECK_TRUE(calibration.valid);
  CHECK_NEAR(calibration.hardIronOffset.x, center.x, 0.5);
  CHECK_NEAR(calibration.hardIronOffset.y, center.y, 0.5);
  CHECK_NEAR(calibration.hardIronOffset.z, center.z, 0.5);

  // 補正後は原点中心になり、水平成分が地磁気相当の大きさを持つこと
  compass::Vec3 probe;
  probe.x = center.x + kRadius;
  probe.y = center.y;
  probe.z = center.z;
  const compass::Vec3 corrected = compass::applyCalibration(calibration, probe);
  CHECK_TRUE(std::hypot(corrected.x, corrected.y) > 20.0F);
}

void testRejectsDegenerateInput() {
  CHECK_TRUE(!compass::fitSphere(nullptr, 100).valid);

  // 4 点未満では解けない
  const compass::Vec3 few[3] = {{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}};
  CHECK_TRUE(!compass::fitSphere(few, 3).valid);

  // 同一平面上に並んだ点は球が決まらない
  std::vector<compass::Vec3> planar;
  for (int index = 0; index < 50; ++index) {
    const float t = static_cast<float>(index);
    planar.push_back(compass::Vec3{t, 2.0F * t, 0.0F});
  }
  CHECK_TRUE(!compass::fitSphere(planar.data(), planar.size()).valid);
}

// 半径の異なる 3 軸を持つ楕円体上の点を作る。rotation が単位行列なら軸に沿う。
std::vector<compass::Vec3> ellipsoidSamples(compass::Vec3 center, compass::Vec3 radii,
                                            const float rotation[3][3], int count) {
  std::vector<compass::Vec3> points;
  points.reserve(static_cast<std::size_t>(count));
  // 球面上をなるべく均等に覆う (黄金角のらせん)。8 の字回しより素直に
  // 全方位を掃くので、当てはめの素性だけを見られる。
  constexpr float kGoldenAngle = 2.39996322972865332F;
  for (int index = 0; index < count; ++index) {
    const float t = (static_cast<float>(index) + 0.5F) / static_cast<float>(count);
    const float elevation = std::asin(2.0F * t - 1.0F);
    const float azimuth = static_cast<float>(index) * kGoldenAngle;

    // 単位球上の点を軸ごとの半径で伸ばす
    const float local[3] = {
        radii.x * std::cos(elevation) * std::cos(azimuth),
        radii.y * std::cos(elevation) * std::sin(azimuth),
        radii.z * std::sin(elevation),
    };

    compass::Vec3 point;
    point.x = center.x + rotation[0][0] * local[0] + rotation[0][1] * local[1] +
              rotation[0][2] * local[2];
    point.y = center.y + rotation[1][0] * local[0] + rotation[1][1] * local[1] +
              rotation[1][2] * local[2];
    point.z = center.z + rotation[2][0] * local[0] + rotation[2][1] * local[1] +
              rotation[2][2] * local[2];
    points.push_back(point);
  }
  return points;
}

// Z 軸まわり yaw、Y 軸まわり pitch の回転行列。歪みを軸から傾けるために使う。
void makeRotation(float yawRadians, float pitchRadians, float rotation[3][3]) {
  const float cosYaw = std::cos(yawRadians);
  const float sinYaw = std::sin(yawRadians);
  const float cosPitch = std::cos(pitchRadians);
  const float sinPitch = std::sin(pitchRadians);

  rotation[0][0] = cosYaw * cosPitch;
  rotation[0][1] = -sinYaw;
  rotation[0][2] = cosYaw * sinPitch;
  rotation[1][0] = sinYaw * cosPitch;
  rotation[1][1] = cosYaw;
  rotation[1][2] = sinYaw * sinPitch;
  rotation[2][0] = -sinPitch;
  rotation[2][1] = 0.0F;
  rotation[2][2] = cosPitch;
}

constexpr float kIdentity[3][3] = {{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}};

void testCorrectsAxisAlignedEllipsoid() {
  // 軸に沿った歪みなら対角補正でも直せるはずの条件。3x3 が退化ケースで
  // 悪化しないことを確かめる。
  const compass::Vec3 center{-125.0F, 14.0F, 340.0F};
  const compass::Vec3 radii{46.0F, 30.0F, 20.0F};
  const std::vector<compass::Vec3> points = ellipsoidSamples(center, radii, kIdentity, 400);

  const compass::EllipsoidFit fit = compass::fitEllipsoid(points.data(), points.size());

  CHECK_TRUE(fit.valid);
  CHECK_TRUE(fit.normalizedResidual < 0.05F);
}

void testCorrectsTiltedEllipsoid() {
  // これが 3x3 を入れた理由。歪みが軸から傾いていると対角では直せない。
  const compass::Vec3 center{-125.0F, 14.0F, 340.0F};
  const compass::Vec3 radii{46.0F, 20.0F, 12.0F};
  float rotation[3][3];
  makeRotation(0.7F, 0.4F, rotation);
  const std::vector<compass::Vec3> points = ellipsoidSamples(center, radii, rotation, 400);

  // 対角補正 (球フィット + 軸ごとのスケール) では残差が残る
  const compass::SphereFit sphere = compass::fitSphere(points.data(), points.size());
  const compass::MagCalibration diagonal =
      compass::calibrationFromSphere(sphere, points.data(), points.size());
  const float diagonalResidual =
      compass::residualAfterCalibration(diagonal, points.data(), points.size());

  // 3x3 なら球に戻る
  const compass::EllipsoidFit fit = compass::fitEllipsoid(points.data(), points.size());

  CHECK_TRUE(fit.valid);
  CHECK_TRUE(fit.normalizedResidual < 0.05F);
  // 対角では直せないこと。これが言えないと 3x3 を入れる意味がない。
  CHECK_TRUE(diagonalResidual > 0.2F);
}

void testRecoversEllipsoidCenter() {
  const compass::Vec3 center{-125.0F, 14.0F, 340.0F};
  const compass::Vec3 radii{46.0F, 20.0F, 12.0F};
  float rotation[3][3];
  makeRotation(0.7F, 0.4F, rotation);
  const std::vector<compass::Vec3> points = ellipsoidSamples(center, radii, rotation, 400);

  const compass::EllipsoidFit fit = compass::fitEllipsoid(points.data(), points.size());

  CHECK_TRUE(fit.valid);
  CHECK_NEAR(fit.center.x, center.x, 1.0);
  CHECK_NEAR(fit.center.y, center.y, 1.0);
  CHECK_NEAR(fit.center.z, center.z, 1.0);
}

void testMatchesFieldConditions() {
  // 実機で観測された条件の再現。ハードアイアンは地磁気 (46uT) の数倍あり、
  // 球フィットの半径は 23uT、対角補正後の残差は 0.414 だった。
  // 軸比と傾きは、その 0.414 が再現される値を選んでいる。
  const compass::Vec3 center{-165.0F, 20.0F, 298.0F};
  constexpr float kBaseRadius = 14.0F;
  const compass::Vec3 radii{kBaseRadius * 2.80F, kBaseRadius, kBaseRadius * 0.35F};
  float rotation[3][3];
  makeRotation(0.7F, 0.4F, rotation);
  const std::vector<compass::Vec3> points = ellipsoidSamples(center, radii, rotation, 500);

  const compass::SphereFit sphere = compass::fitSphere(points.data(), points.size());
  const compass::MagCalibration diagonal =
      compass::calibrationFromSphere(sphere, points.data(), points.size());
  const float diagonalResidual =
      compass::residualAfterCalibration(diagonal, points.data(), points.size());

  const compass::EllipsoidFit fit = compass::fitEllipsoid(points.data(), points.size());

  CHECK_TRUE(fit.valid);
  // 球フィットの半径が実機と同じ 23uT 規模であること
  CHECK_NEAR(sphere.radius, 23.0F, 2.0);
  // 対角補正では実機と同じ 0.414 が残ること。これが再現できていないと
  // 「3x3 でどこまで下がるか」の比較に意味がない。
  CHECK_NEAR(diagonalResidual, 0.414F, 0.02);
  // 3x3 ならコンパスとして使える水準まで落ちること
  CHECK_TRUE(fit.normalizedResidual < 0.1F);
  CHECK_NEAR(fit.meanRadius, kBaseRadius * (2.80F + 1.0F + 0.35F) / 3.0F, 1.0);

  std::printf("  field-condition residual: diagonal=%.3f ellipsoid=%.4f\n",
              static_cast<double>(diagonalResidual), static_cast<double>(fit.normalizedResidual));
}

void testRejectsDegenerateEllipsoidInput() {
  CHECK_TRUE(!compass::fitEllipsoid(nullptr, 100).valid);

  // 未知数 9 個に対して点が足りない
  const std::vector<compass::Vec3> few = ellipsoidSamples(
      compass::Vec3{0.0F, 0.0F, 0.0F}, compass::Vec3{40.0F, 30.0F, 20.0F}, kIdentity, 9);
  CHECK_TRUE(!compass::fitEllipsoid(few.data(), few.size()).valid);

  // 同一平面上の点は楕円体が決まらない
  std::vector<compass::Vec3> planar;
  for (int index = 0; index < 100; ++index) {
    const float angle = static_cast<float>(index) * 0.1F;
    planar.push_back(compass::Vec3{40.0F * std::cos(angle), 30.0F * std::sin(angle), 5.0F});
  }
  CHECK_TRUE(!compass::fitEllipsoid(planar.data(), planar.size()).valid);
}

} // namespace

int main() {
  testRecoversKnownSphere();
  testSucceedsWherePeakToPeakFails();
  testCalibrationFromSphere();
  testRejectsDegenerateInput();
  testCorrectsAxisAlignedEllipsoid();
  testCorrectsTiltedEllipsoid();
  testRecoversEllipsoidCenter();
  testMatchesFieldConditions();
  testRejectsDegenerateEllipsoidInput();
  return testing::summarize("sphere-fit");
}
