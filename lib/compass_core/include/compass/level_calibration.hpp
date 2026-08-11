#pragma once

#include "compass/calibration.hpp"

#include <cstddef>

namespace compass {

// 本体を水平に一回転させて、水平面 (X-Y) の円だけを当てはめる。
//
// Why not 3 軸を振り回す 8 の字回し: この機体は本体そのものに強い磁石を
// 持っており (ハードアイアンが地磁気の 6 倍)、デバイスを傾けると磁石も
// 一緒に動く。実機で 8 の字回しをしたところ、球の半径が 169uT (地磁気の
// 3.7 倍) になり、地磁気ではなく磁石が回転して描く軌跡を見ていた。
// 楕円体の当てはめも解けない。
//
// 一方、首を固定して本体だけを水平に回せば、磁石との相対関係が保たれる。
// 実機で首を固定して測ったときの方位のばらつきは 1.9 度で、地磁気は
// きちんと読めていた。
//
// 生センサーX/Yへ直接当てるのではなく、加速度計の姿勢で水平座標へ投影した
// X/Yに対して使う。そうすれば傾いたCoreS3でも、固定磁石は一定の中心、
// 地磁気の水平成分は円として観測できる。
struct LevelCalibration {
  // 水平投影後のハードアイアン。
  float offsetX = 0.0F;
  float offsetY = 0.0F;
  // X と Y の感度差。crossAxisも使い、軸に対して回転した楕円も円へ戻す。
  float scaleX = 1.0F;
  float scaleY = 1.0F;
  float crossAxis = 0.0F;
  float radius = 0.0F;
  // 円にどれだけ乗っているか。0 に近いほど良い。
  float normalizedResidual = 1.0F;
  bool valid = false;
};

// 水平回転で集めた点から円を当てはめる。
// 点が円周に十分散らばっていないと invalid を返す。
[[nodiscard]] LevelCalibration fitLevelCircle(const Vec3* points, std::size_t count);

// 水平投影済みのX/Yへ補正を当てる。Zは方位計算に使わない。
[[nodiscard]] Vec3 applyLevelCalibration(const LevelCalibration& calibration, Vec3 raw);

// 集めた点が円周のどれだけを覆っているか [0..1]。
// 一回転すれば 1 に近づく。UI の進捗表示と採否の判定に使う。
[[nodiscard]] float angularCoverage(const Vec3* points, std::size_t count, float centerX,
                                    float centerY);

} // namespace compass
