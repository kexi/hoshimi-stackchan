#pragma once

namespace compass {

// 磁気偏角の扱い。真方位 = 磁方位 + 偏角 (東偏を正)。
//
// 内部計算はすべて真方位に統一する。天体計算が返すのは真方位なので、
// センサの磁方位をそのまま使うと日本では 7-9 度ずれる。方位精度 0.1 度の
// 要求に対して致命的なので、偏角の設定は必須。
//
// Why not IGRF 全球モデル: 13 次の係数表 (195 係数 x 2) を積むほどの価値がない。
// ユーザーが自分の土地の値を NOAA の計算機から一度入れれば 0.01 度精度で足りる。
[[nodiscard]] float trueHeadingFromMagnetic(float magneticHeadingDegrees,
                                            float declinationEastDegrees);

[[nodiscard]] float magneticHeadingFromTrue(float trueHeadingDegrees, float declinationEastDegrees);

} // namespace compass
