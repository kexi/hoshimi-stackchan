#pragma once

#include "astro/coords.hpp"
#include "astro/time.hpp"

namespace astro {

// 太陽の見かけの地心黄道座標。Meeus ch.25 の低精度式に中心差と光行差を加えたもの。
// 期待精度: 黄経誤差 < 0.01 度 (1950-2050)。方位精度 0.1 度の要求に対して十分。
[[nodiscard]] EclipticCoord sunEcliptic(JulianDate julianDate);

// 見かけの地心赤道座標 (章動込みの真黄道傾斜角で変換)。
[[nodiscard]] EquatorialCoord sunEquatorial(JulianDate julianDate);

} // namespace astro
