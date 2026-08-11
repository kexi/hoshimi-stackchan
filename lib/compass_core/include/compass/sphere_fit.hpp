#pragma once

#include "compass/calibration.hpp"

#include <cstddef>

namespace compass {

// 磁気サンプルから球の中心と半径を最小二乗で求める。
//
// Why not min/max: min/max は「その軸の端に到達した 2 点」しか使わないので、
// 端まで回しきらないと中心が回した範囲の中心に寄る。実機ではそれで
// 補正後の水平成分が地磁気の 6 割になり、方位が全方位に散らばった。
//
// 最小二乗なら全サンプルが中心の推定に寄与するので、端に到達していなくても
// 球面の一部さえ掃ければ中心が求まる。結果として短い回転で済む。
//
// 解法: |p - c|^2 = r^2 を展開すると
//   2*cx*x + 2*cy*y + 2*cz*z + (r^2 - |c|^2) = x^2 + y^2 + z^2
// となり、未知数 (cx, cy, cz, k) について線形。正規方程式 4x4 をガウスの
// 消去法で解く。サンプル数に依らず固定サイズなので ESP32 でも軽い。
struct SphereFit {
  Vec3 center;
  float radius = 0.0F;
  // 残差の RMS を半径で割った値。0 に近いほど球に乗っている。
  float normalizedResidual = 1.0F;
  bool valid = false;
};

// サンプル列から球を当てはめる。points が null か count が 4 未満なら invalid。
[[nodiscard]] SphereFit fitSphere(const Vec3* points, std::size_t count);

// 対角ソフトアイアン補正を当てた後で、どれだけ球に乗っているかを測る。
//
// 生の磁場は楕円体になっているのが普通で (実機では Z の広がりが X の 2.5 倍)、
// 補正前の残差で採否を決めると正常なデータまで弾いてしまう。補正は
// まさにこの歪みを直すためのものなので、補正後の姿で評価する。
[[nodiscard]] float residualAfterCalibration(const MagCalibration& calibration, const Vec3* points,
                                             std::size_t count);

// 任意の向きの楕円体を球へ戻す 3x3 のソフトアイアン補正。
//
// Why not 対角行列: 対角は軸に沿った伸び縮みしか直せない。実機では対角補正を
// 当てても残差 0.41 が残り、方位に直すと 32 度の誤差になった。歪みが軸から
// 傾いているため。3x3 なら傾いた楕円体も球に戻せる。
//
// Why not 当初の判断 (過適合を懸念して対角に留める): 実機の歪みが想定より
// 大きく、対角では方位が使い物にならなかった。過適合のリスクより、
// 直せないことの害の方が大きい。
struct EllipsoidFit {
  Vec3 center;
  // 補正行列。corrected = matrix * (raw - center)
  float matrix[3][3] = {{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}};
  float meanRadius = 0.0F;
  float normalizedResidual = 1.0F;
  bool valid = false;
};

[[nodiscard]] EllipsoidFit fitEllipsoid(const Vec3* points, std::size_t count);
[[nodiscard]] Vec3 applyEllipsoid(const EllipsoidFit& fit, Vec3 raw);

// 球の当てはめ結果からキャリブレーションを作る。
//
// 軸ごとのスケール (対角ソフトアイアン) は、中心を引いた後の各軸の
// 二乗平均から求める。等方な球ならすべて 1 になる。
[[nodiscard]] MagCalibration calibrationFromSphere(const SphereFit& fit, const Vec3* points,
                                                   std::size_t count);

} // namespace compass
