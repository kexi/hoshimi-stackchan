#pragma once

#include "compass/calibration.hpp"

#include <cstddef>

namespace compass {

// 機体の傾き。加速度計から求める。
struct Attitude {
  float rollRadians = 0.0F;
  float pitchRadians = 0.0F;
};

// 重力ベクトルから roll/pitch を出す。静止していることが前提。
[[nodiscard]] Attitude attitudeFromAccel(Vec3 accel);

// 機体座標の磁気を、重力に垂直な水平X/Yへ投影する。
// 水平回転キャリブレーションは、この座標で円を当てはめる。
[[nodiscard]] Vec3 horizontalMagneticComponents(Vec3 magneticField, Attitude attitude);

// 水平X/Yから磁北基準の方位を出す [度, 0..360)。
[[nodiscard]] float headingDegreesFromHorizontal(Vec3 horizontalMagneticField);

// 傾斜補正済みの磁気方位 [度, 0..360)。磁北基準。
// mag はキャリブレーション適用済みの値を渡すこと。
[[nodiscard]] float tiltCompensatedHeadingDegrees(Vec3 calibratedMag, Attitude attitude);

// 角度を 0..360 に畳む。
[[nodiscard]] float normalizeDegrees(float degrees);

// from から to への符号つき角度差 [度, -180..180]。
// 方位は円環量なので単純な引き算だと 359 と 1 の差が 358 になる。
[[nodiscard]] float signedAngleDifference(float toDegrees, float fromDegrees);

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

  // 取り込んだサンプル数。収束の判断に使う。
  //
  // 指数移動平均は数サンプルでは初期値に引きずられる。これを見ずに判定すると、
  // 収束前の値を「測れた」と誤って採用する (実機で方位が ±40 度揺れた原因)。
  [[nodiscard]] std::size_t sampleCount() const;
  [[nodiscard]] bool hasConverged(std::size_t minimumSamples) const;

private:
  float smoothing_;
  float sumSin_ = 0.0F;
  float sumCos_ = 0.0F;
  bool hasValue_ = false;
  std::size_t sampleCount_ = 0;
};

} // namespace compass
