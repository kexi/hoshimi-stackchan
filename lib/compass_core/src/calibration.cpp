#include "compass/calibration.hpp"

#include <algorithm>
#include <cmath>

namespace compass {
namespace {

float axisRange(float minValue, float maxValue) { return maxValue - minValue; }

} // namespace

float magnitude(Vec3 value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 applyCalibration(const MagCalibration& calibration, Vec3 raw) {
  if (!calibration.valid) {
    return raw;
  }
  Vec3 result;
  result.x = (raw.x - calibration.hardIronOffset.x) * calibration.softIronScale.x;
  result.y = (raw.y - calibration.hardIronOffset.y) * calibration.softIronScale.y;
  result.z = (raw.z - calibration.hardIronOffset.z) * calibration.softIronScale.z;
  return result;
}

CalibrationCollector::CalibrationCollector(CalibrationConfig config) : config_(config) {}

void CalibrationCollector::reset() {
  min_ = Vec3{};
  max_ = Vec3{};
  count_ = 0;
  hasSample_ = false;
}

void CalibrationCollector::addSample(Vec3 raw) {
  if (!hasSample_) {
    min_ = raw;
    max_ = raw;
    hasSample_ = true;
  } else {
    min_.x = std::min(min_.x, raw.x);
    min_.y = std::min(min_.y, raw.y);
    min_.z = std::min(min_.z, raw.z);
    max_.x = std::max(max_.x, raw.x);
    max_.y = std::max(max_.y, raw.y);
    max_.z = std::max(max_.z, raw.z);
  }
  ++count_;
}

std::size_t CalibrationCollector::sampleCount() const { return count_; }

float CalibrationCollector::coverage() const {
  if (!hasSample_ || config_.minAxisRangeMicroTesla <= 0.0F || config_.minSamples == 0) {
    return 0.0F;
  }

  // サンプル数と、3 軸それぞれのレンジ充足率のうち最も低いものを進捗とする。
  // 一番遅れている条件を見せた方が「あと何をすればよいか」が伝わる。
  const float sampleProgress = static_cast<float>(count_) / static_cast<float>(config_.minSamples);
  const float rangeProgress =
      std::min({axisRange(min_.x, max_.x), axisRange(min_.y, max_.y), axisRange(min_.z, max_.z)}) /
      config_.minAxisRangeMicroTesla;

  return std::min(1.0F, std::min(sampleProgress, rangeProgress));
}

MagCalibration CalibrationCollector::finish() const {
  MagCalibration calibration;
  if (!hasSample_ || count_ < config_.minSamples) {
    return calibration;
  }

  const float rangeX = axisRange(min_.x, max_.x);
  const float rangeY = axisRange(min_.y, max_.y);
  const float rangeZ = axisRange(min_.z, max_.z);
  const float smallestRange = std::min({rangeX, rangeY, rangeZ});
  const float largestRange = std::max({rangeX, rangeY, rangeZ});

  // 回し足りない軸があるまま採用すると、その軸のスケールが暴れて方位が破綻する。
  if (smallestRange < config_.minAxisRangeMicroTesla) {
    return calibration;
  }
  if (smallestRange <= 0.0F || largestRange / smallestRange > config_.maxAxisRangeRatio) {
    return calibration;
  }

  calibration.hardIronOffset.x = 0.5F * (min_.x + max_.x);
  calibration.hardIronOffset.y = 0.5F * (min_.y + max_.y);
  calibration.hardIronOffset.z = 0.5F * (min_.z + max_.z);

  // 各軸の半径を平均半径に揃える (対角ソフトアイアン)。
  const float meanRadius = (rangeX + rangeY + rangeZ) / 6.0F;
  calibration.softIronScale.x = meanRadius / (0.5F * rangeX);
  calibration.softIronScale.y = meanRadius / (0.5F * rangeY);
  calibration.softIronScale.z = meanRadius / (0.5F * rangeZ);
  calibration.valid = true;
  return calibration;
}

} // namespace compass
