#pragma once

#include "astro/coords.hpp"
#include "astro/time.hpp"

namespace astro {

// 月の地心黄道座標。Meeus ch.47 (ELP2000-82 の短縮版、黄経 60 項 / 黄緯 60 項 / 距離 60 項)。
// 期待精度: 黄経 10 秒角 (0.003 度) 程度。
//
// これは地心座標なので、地上から見た方向としては最大 1 度ずれる。
// 観測地の方向を出すときは必ず applyTopocentricParallax() を通すこと。
[[nodiscard]] EclipticCoord moonEcliptic(JulianDate julianDate);

// 地心赤道座標。測心への変換は呼び出し側が行う。
[[nodiscard]] EquatorialCoord moonEquatorial(JulianDate julianDate);

} // namespace astro
