#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace compass {

// 磁気測定を「信じてよい」瞬間だけ通すゲート。
//
// 実機で測った結果 (段階 8、CoreS3 + M5 公式スタックチャン基板):
//   - サーボの通電状態による差は |B| で 3.0uT、方位で 2.4 度。
//     電源 OFF / トルク OFF / トルク ON のどれでもほぼ変わらない。
//     → 測定のたびにサーボ電源を落とす必要はない
//   - 首の角度による方位のばらつきは 119 度。これが支配的な誤差要因。
//     ハードアイアン (地磁気の約 6 倍) が首と一緒に回るため。
//     → 測定は必ず首を正面 (yaw=0) に戻してから行う。ServoBiasTable より
//       確実で、実装も単純
//   - サーボ停止から方位が収まるまでは 300ms
struct MeasurementGateConfig {
  // サーボ停止後、磁場が落ち着くまで捨てる時間。実測 300ms に余裕を持たせた値。
  std::uint32_t settleMillis = 400;
  // 機体自体が動かされていないことの判定。
  float maxGyroDegPerSec = 3.0F;
  // 静穏時の |B| からのずれ許容率。実測では通電で 8% しか動かないので、
  // ここに掛かるのは磁石を近づけられたような明らかな外乱だけ。
  float maxFieldDeviationRatio = 0.25F;
  // 実測に基づく値。
  //
  // サーボ電源を切った状態では 1.9 度に収まるが、通電したままだと段階 8 で
  // 測った 2.4 度ぶんの振れが乗り、実機では 4.7 度まで出た。通電中も測る
  // 設計 (層 3 は不要と判断した) なので、そこを含めた値にする。
  //
  // 6 度は「サーボのノイズは通すが、磁石を近づけられたような外乱は弾く」
  // 水準。ここを詰めるより、首を正面に戻す層 1 の方が効果が大きい
  // (角度による誤差は 119 度あった)。
  float maxHeadingDispersionDegrees = 6.0F;
};

// 磁気を測ってよい首の姿勢。ここから外れた角度で測ると、首と一緒に回る
// ハードアイアンのせいで最大 119 度ずれる (段階 8 実測)。
inline constexpr int kMeasurementYawDeci = 0;
// 正面からこの範囲に入っていれば測ってよい。
//
// 実測の傾きは 16 ビンで 119 度、つまり 1 ビン (16 度) あたり約 7 度。
// ±5 度なら首の角度による誤差は 2 度程度に収まる。
//
// Why not ±2 度: サーボが正面へ戻ったときの静定位置は実測で -2.1 度あり、
// 許容範囲の境界に張り付いた。0.1 度の超過で測定が弾かれ、フィルタが
// 空のまま Measuring から抜けられなくなる (実機で発生)。
// サーボの静定精度に対して余裕を持たせる。
inline constexpr int kMeasurementYawToleranceDeci = 50;

[[nodiscard]] bool isMeasurementPose(int yawDeciDegrees);

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
    // 測定時の首の角度。正面から外れていると採用しない。
    int yawDeciDegrees = 0;
  };

  enum class Reject : std::uint8_t {
    None = 0,
    ServoMoving,
    Settling,
    DeviceMoving,
    FieldAnomaly,
    Unstable,
    // 首が正面にない。実測では最大の誤差要因なので最優先で弾く。
    NotMeasurementPose,
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
