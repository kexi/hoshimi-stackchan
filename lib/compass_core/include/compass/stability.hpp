#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace compass {

// 磁気測定を「信じてよい」瞬間だけ通すゲート。
//
// BMM150 は本体基板上にあり、サーボは至近距離にある。首を振りながら測ると
// コイル電流の作る磁場が地磁気 (日本で約 46uT) に乗って方位が破綻する。
// このゲートがこのプロジェクトの成否を分ける。
struct MeasurementGateConfig {
  // サーボ停止後、磁場が落ち着くまで捨てる時間。実測 (段階 8) で詰める。
  std::uint32_t settleMillis = 400;
  // 機体自体が動かされていないことの判定。
  float maxGyroDegPerSec = 3.0F;
  // 静穏時の |B| からのずれ許容率。保持電流が流れていると |B| がずれる。
  float maxFieldDeviationRatio = 0.25F;
  float maxHeadingDispersionDegrees = 3.0F;
};

class MeasurementGate {
public:
  using Config = MeasurementGateConfig;

  struct Input {
    std::uint32_t nowMillis = 0;
    bool servoMoving = false;
    std::uint32_t lastServoStopMillis = 0;
    float gyroMagnitudeDegPerSec = 0.0F;
    float fieldMagnitudeMicroTesla = 0.0F;
    float headingDispersionDegrees = 0.0F;
  };

  enum class Reject : std::uint8_t {
    None = 0,
    ServoMoving,
    Settling,
    DeviceMoving,
    FieldAnomaly,
    Unstable,
  };

  explicit MeasurementGate(MeasurementGateConfig config = {});

  // 静穏時の |B| を学習する。採用されたサンプルだけを食わせること。
  void learnReferenceField(float fieldMagnitudeMicroTesla);
  [[nodiscard]] float referenceFieldMicroTesla() const;
  [[nodiscard]] bool hasReferenceField() const;
  void resetReferenceField();

  [[nodiscard]] Reject evaluate(const Input& input) const;
  [[nodiscard]] bool accepts(const Input& input) const;

private:
  MeasurementGateConfig config_;
  float referenceField_ = 0.0F;
  bool hasReference_ = false;
};

[[nodiscard]] const char* rejectName(MeasurementGate::Reject reject);

// サーボ角度に依存する磁気バイアスの補正表。
//
// ゲート (層 1-3) を通してなお、首の「位置」による静的バイアスが残る。
// サーボの永久磁石そのものが動くためで、トルクを切っても消えない。
// yaw をビンに離散化し、機体を固定したまま首だけ振ったときの方位測定値の
// ずれを学習する (機体が動いていないなら真方位は全ビンで同一のはず)。
class ServoBiasTable {
public:
  static constexpr std::size_t kBinCount = 16;
  static constexpr int kYawMinDeci = -1280;
  static constexpr int kYawMaxDeci = 1280;

  void reset();
  void observe(int yawDeciDegrees, float headingErrorDegrees);
  // 学習済みのビンが無ければ 0 を返す (補正しない)。
  [[nodiscard]] float correctionDegrees(int yawDeciDegrees) const;
  [[nodiscard]] bool isPopulated() const;
  [[nodiscard]] std::size_t populatedBinCount() const;

  [[nodiscard]] static std::size_t binIndexFor(int yawDeciDegrees);

private:
  std::array<float, kBinCount> correction_{};
  std::array<std::uint16_t, kBinCount> observationCount_{};
};

} // namespace compass
