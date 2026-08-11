#include "compass/heading.hpp"

#include <algorithm>
#include <cmath>

namespace compass {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kRad2Deg = 180.0F / kPi;
constexpr float kDeg2Rad = kPi / 180.0F;

float normalizeDegrees(float degrees) {
  float wrapped = std::fmod(degrees, 360.0F);
  if (wrapped < 0.0F) {
    wrapped += 360.0F;
  }
  if (wrapped >= 360.0F) {
    wrapped = 0.0F;
  }
  return wrapped;
}

} // namespace

Attitude attitudeFromAccel(Vec3 accel) {
  Attitude attitude;
  // roll は X 軸まわり、pitch は Y 軸まわり。加速度が重力のみと仮定する。
  attitude.rollRadians = std::atan2(accel.y, accel.z);
  attitude.pitchRadians = std::atan2(-accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
  return attitude;
}

float tiltCompensatedHeadingDegrees(Vec3 calibratedMag, Attitude attitude) {
  const float sinRoll = std::sin(attitude.rollRadians);
  const float cosRoll = std::cos(attitude.rollRadians);
  const float sinPitch = std::sin(attitude.pitchRadians);
  const float cosPitch = std::cos(attitude.pitchRadians);

  // 機体座標の磁場を水平面へ倒し込む。
  const float horizontalX = calibratedMag.x * cosPitch + calibratedMag.y * sinRoll * sinPitch +
                            calibratedMag.z * cosRoll * sinPitch;
  const float horizontalY = calibratedMag.y * cosRoll - calibratedMag.z * sinRoll;

  return normalizeDegrees(std::atan2(-horizontalY, horizontalX) * kRad2Deg);
}

HeadingFilter::HeadingFilter(float smoothingFactor)
    : smoothing_(std::clamp(smoothingFactor, 0.01F, 1.0F)) {}

void HeadingFilter::reset() {
  sumSin_ = 0.0F;
  sumCos_ = 0.0F;
  hasValue_ = false;
  sampleCount_ = 0;
}

std::size_t HeadingFilter::sampleCount() const { return sampleCount_; }

bool HeadingFilter::hasConverged(std::size_t minimumSamples) const {
  return hasValue_ && sampleCount_ >= minimumSamples;
}

void HeadingFilter::update(float headingDegrees) {
  const float radians = headingDegrees * kDeg2Rad;
  const float sinValue = std::sin(radians);
  const float cosValue = std::cos(radians);

  ++sampleCount_;

  if (!hasValue_) {
    sumSin_ = sinValue;
    sumCos_ = cosValue;
    hasValue_ = true;
    return;
  }

  sumSin_ += smoothing_ * (sinValue - sumSin_);
  sumCos_ += smoothing_ * (cosValue - sumCos_);
}

bool HeadingFilter::hasValue() const { return hasValue_; }

float HeadingFilter::valueDegrees() const {
  if (!hasValue_) {
    return 0.0F;
  }
  return normalizeDegrees(std::atan2(sumSin_, sumCos_) * kRad2Deg);
}

float HeadingFilter::dispersionDegrees() const {
  if (!hasValue_) {
    // 「まだ値が無い」は「ばらついている」とは違う。
    // 180 を返すと、これを見るゲートが Unstable で弾いてしまい、
    // 値が溜まる前に測定が打ち切られる (実機で方位が確定しなかった一因)。
    // 呼び出し側は hasValue() で有無を判断すること。
    return 0.0F;
  }
  // 平均ベクトルの長さ R は、値が揃っていれば 1、散らばるほど 0 に近づく。
  // 円周分散から標準偏差相当を出す (Mardia の circular standard deviation)。
  const float resultantLength = std::min(1.0F, std::sqrt(sumSin_ * sumSin_ + sumCos_ * sumCos_));
  if (resultantLength <= 1e-6F) {
    return 180.0F;
  }
  return std::sqrt(-2.0F * std::log(resultantLength)) * kRad2Deg;
}

} // namespace compass
