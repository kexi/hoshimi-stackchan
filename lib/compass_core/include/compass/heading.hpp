#pragma once

#include "compass/calibration.hpp"

namespace compass {

// 機体の傾き。加速度計から求める。
struct Attitude {
  float rollRadians = 0.0F;
  float pitchRadians = 0.0F;
};

// 重力ベクトルから roll/pitch を出す。静止していることが前提。
[[nodiscard]] Attitude attitudeFromAccel(Vec3 accel);

// 傾斜補正済みの磁気方位 [度, 0..360)。磁北基準。
// mag はキャリブレーション適用済みの値を渡すこと。
[[nodiscard]] float tiltCompensatedHeadingDegrees(Vec3 calibratedMag, Attitude attitude);

// 方位の指数移動平均。
//
// Why not 単純な移動平均: 方位は円環量なので 359 度と 1 度の平均が 180 度に
// なってしまう。sin/cos に分解してから平均し、atan2 で戻す。
class HeadingFilter {
public:
  explicit HeadingFilter(float smoothingFactor = 0.2F);

  void reset();
  void update(float headingDegrees);

  [[nodiscard]] bool hasValue() const;
  [[nodiscard]] float valueDegrees() const;
  // 直近のばらつき [度]。合成ベクトルの長さが 1 から離れるほど散らばっている。
  [[nodiscard]] float dispersionDegrees() const;

private:
  float smoothing_;
  float sumSin_ = 0.0F;
  float sumCos_ = 0.0F;
  bool hasValue_ = false;
};

} // namespace compass
