#pragma once

#include <cstddef>

namespace compass {

struct Vec3 {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

[[nodiscard]] float magnitude(Vec3 value);

// ハードアイアン (オフセット) と対角ソフトアイアン (軸ごとのスケール)。
//
// Why not 完全な 3x3 ソフトアイアン行列: 8 の字回しでは姿勢の多様性が足りず、
// 非対角成分まで推定すると過適合してキャリブレーション時の姿勢に依存した
// 誤差が乗る。対角までに留めた方が実機では安定する。
struct MagCalibration {
  Vec3 hardIronOffset;
  Vec3 softIronScale{1.0F, 1.0F, 1.0F};
  bool valid = false;
};

[[nodiscard]] Vec3 applyCalibration(const MagCalibration& calibration, Vec3 raw);

// 採用条件。クラス外に置くのは、既定引数 `= {}` がクラス定義の途中では
// メンバ初期化子を参照できないため (C++ の制約)。
struct CalibrationConfig {
  std::size_t minSamples = 200;
  float minAxisRangeMicroTesla = 20.0F;
  // 最長軸 / 最短軸。これを超えたら回し方が偏っていると見なす。
  float maxAxisRangeRatio = 3.0F;
};

// 8 の字回し中のサンプルから min/max を集め、中心とスケールを出す。
class CalibrationCollector {
public:
  using Config = CalibrationConfig;

  explicit CalibrationCollector(CalibrationConfig config = {});

  void reset();
  void addSample(Vec3 raw);

  [[nodiscard]] std::size_t sampleCount() const;
  // UI の進捗表示用。0..1 で、1 になれば finish() が valid を返せる見込み。
  [[nodiscard]] float coverage() const;
  [[nodiscard]] MagCalibration finish() const;

private:
  CalibrationConfig config_;
  Vec3 min_;
  Vec3 max_;
  std::size_t count_ = 0;
  bool hasSample_ = false;
};

} // namespace compass
