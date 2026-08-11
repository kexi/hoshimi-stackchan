#pragma once

namespace compass {

// 磁気偏角の扱い。真方位 = 磁方位 + 偏角 (東偏を正)。
//
// 内部計算はすべて真方位に統一する。天体計算が返すのは真方位なので、
// センサの磁方位をそのまま使うと日本では 7-9 度ずれる。方位精度 0.1 度の
// 要求に対して致命的なので、偏角の設定は必須。
//
// Why not IGRF 全球モデル: 13次の係数表を実機へ積むより、設置場所と利用年に
// 対応する値を設定ファイルで更新する方が小さい。位置や年を変える場合は再設定する。
[[nodiscard]] float trueHeadingFromMagnetic(float magneticHeadingDegrees,
                                            float declinationEastDegrees);

[[nodiscard]] float magneticHeadingFromTrue(float trueHeadingDegrees, float declinationEastDegrees);

} // namespace compass
