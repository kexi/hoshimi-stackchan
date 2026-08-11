#include "compass/stability.hpp"

#include <algorithm>
#include <cmath>

namespace compass {
namespace {

// 円環量の差 (-180, 180]
float angleDifference(float fromDegrees, float toDegrees) {
  float difference = std::fmod(toDegrees - fromDegrees, 360.0F);
  if (difference > 180.0F) {
    difference -= 360.0F;
  }
  if (difference <= -180.0F) {
    difference += 360.0F;
  }
  return difference;
}

} // namespace

bool isMeasurementPose(int yawDeciDegrees) {
  return yawDeciDegrees >= kMeasurementYawDeci - kMeasurementYawToleranceDeci &&
         yawDeciDegrees <= kMeasurementYawDeci + kMeasurementYawToleranceDeci;
}

MeasurementGate::MeasurementGate(MeasurementGateConfig config) : config_(config) {}

void MeasurementGate::learnReferenceField(float fieldMagnitudeMicroTesla) {
  if (!(fieldMagnitudeMicroTesla > 0.0F)) {
    return;
  }
  if (!hasReference_) {
    referenceField_ = fieldMagnitudeMicroTesla;
    hasReference_ = true;
    return;
  }
  // ゆっくり追従させる。急に引っ張られると異常値を基準にしてしまう。
  referenceField_ += 0.05F * (fieldMagnitudeMicroTesla - referenceField_);
}

float MeasurementGate::referenceFieldMicroTesla() const { return referenceField_; }

bool MeasurementGate::hasReferenceField() const { return hasReference_; }

void MeasurementGate::resetReferenceField() {
  referenceField_ = 0.0F;
  hasReference_ = false;
}

MeasurementGate::Reject MeasurementGate::evaluate(const Input& input) const {
  // 首が正面にないと最大 119 度ずれる (段階 8 実測)。他のどの要因より大きいので
  // 最初に見る。
  //
  // ただし ServoBiasTable が学習済みなら、その角度のずれは打ち消せている。
  // 首を正面へ戻さずに測れないと、持ち歩きながら指し続けることができない。
  if (!input.biasCorrected && !isMeasurementPose(input.yawDeciDegrees)) {
    return Reject::NotMeasurementPose;
  }

  if (input.servoMoving) {
    return Reject::ServoMoving;
  }

  // 停止直後は磁場がまだ揺れている。経過時間は wrap 安全な引き算で見る。
  const std::uint32_t sinceStop = input.nowMillis - input.lastServoStopMillis;
  if (sinceStop < config_.settleMillis) {
    return Reject::Settling;
  }

  if (input.gyroMagnitudeDegPerSec > config_.maxGyroDegPerSec) {
    return Reject::DeviceMoving;
  }

  // |B| が基準から外れていれば、静止していてもサーボの保持電流が疑われる。
  if (hasReference_ && referenceField_ > 0.0F) {
    const float deviation =
        std::fabs(input.fieldMagnitudeMicroTesla - referenceField_) / referenceField_;
    if (deviation > config_.maxFieldDeviationRatio) {
      return Reject::FieldAnomaly;
    }
  }

  if (input.headingDispersionDegrees > config_.maxHeadingDispersionDegrees) {
    return Reject::Unstable;
  }

  return Reject::None;
}

bool MeasurementGate::accepts(const Input& input) const { return evaluate(input) == Reject::None; }

const char* rejectName(MeasurementGate::Reject reject) {
  switch (reject) {
  case MeasurementGate::Reject::None:
    return "none";
  case MeasurementGate::Reject::ServoMoving:
    return "servo-moving";
  case MeasurementGate::Reject::Settling:
    return "settling";
  case MeasurementGate::Reject::DeviceMoving:
    return "device-moving";
  case MeasurementGate::Reject::FieldAnomaly:
    return "field-anomaly";
  case MeasurementGate::Reject::NotMeasurementPose:
    return "not-measurement-pose";
  case MeasurementGate::Reject::Unstable:
    break;
  }
  return "unstable";
}

std::size_t ServoBiasTable::binIndexFor(int yawDeciDegrees) {
  const int clamped = std::clamp(yawDeciDegrees, kYawMinDeci, kYawMaxDeci);
  const long span = static_cast<long>(kYawMaxDeci) - kYawMinDeci;
  const long offset = static_cast<long>(clamped) - kYawMinDeci;
  const long index = (offset * static_cast<long>(kBinCount)) / span;
  return static_cast<std::size_t>(std::clamp<long>(index, 0, kBinCount - 1));
}

void ServoBiasTable::reset() {
  correction_.fill(0.0F);
  observationCount_.fill(0);
}

bool ServoBiasTable::observe(int yawDeciDegrees, float headingErrorDegrees) {
  const std::size_t index = binIndexFor(yawDeciDegrees);
  const std::uint16_t previousCount = observationCount_[index];

  // 累積平均。飽和を避けるため回数は上限で止める。
  if (previousCount == 0) {
    correction_[index] = headingErrorDegrees;
    observationCount_[index] = 1;
    return true;
  }
  if (previousCount < 1000) {
    observationCount_[index] = static_cast<std::uint16_t>(previousCount + 1);
  }
  const float weight = 1.0F / static_cast<float>(observationCount_[index]);
  // 誤差も円環量なので、単純な差ではなく最短角差で寄せる。
  correction_[index] += weight * angleDifference(correction_[index], headingErrorDegrees);

  // 1,2,4,...回目だけを永続化候補にする。毎観測をdirtyにすると、静止中に
  // 30秒ごと永続的にNVSへ書き続ける。標本が増えるほど保存を間引いても、
  // 再起動後に最初の1点だけへ戻る問題は避けられる。
  const std::uint16_t currentCount = observationCount_[index];
  const bool isPowerOfTwo = (currentCount & static_cast<std::uint16_t>(currentCount - 1)) == 0;
  const bool reachedMaximum = currentCount == 1000 && previousCount < 1000;
  return isPowerOfTwo || reachedMaximum;
}

float ServoBiasTable::correctionDegrees(int yawDeciDegrees) const {
  const std::size_t index = binIndexFor(yawDeciDegrees);
  if (observationCount_[index] > 0) {
    return correction_[index];
  }

  // 未学習のビンは、両隣で学習済みのものがあれば近い方の値を借りる。
  // Why not 線形補間: 学習済みビンが疎なときに、遠いビンの値を引き伸ばして
  // しまい、実測していない領域で誤差を増やす方向に働く。
  for (std::size_t distance = 1; distance < kBinCount; ++distance) {
    if (index >= distance && observationCount_[index - distance] > 0) {
      return correction_[index - distance];
    }
    if (index + distance < kBinCount && observationCount_[index + distance] > 0) {
      return correction_[index + distance];
    }
  }
  return 0.0F;
}

bool ServoBiasTable::isPopulated() const { return populatedBinCount() >= kMinPopulatedBins; }

bool ServoBiasTable::hasObservationFor(int yawDeciDegrees) const {
  return observationCount_[binIndexFor(yawDeciDegrees)] > 0;
}

std::size_t ServoBiasTable::populatedBinCount() const {
  std::size_t populated = 0;
  for (const std::uint16_t count : observationCount_) {
    if (count > 0) {
      ++populated;
    }
  }
  return populated;
}

} // namespace compass
