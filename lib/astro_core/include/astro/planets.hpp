#pragma once

#include "astro/coords.hpp"
#include "astro/time.hpp"

namespace astro {

enum class Planet { Mercury, Venus, Earth, Mars, Jupiter, Saturn };

// J2000 黄道を基準とした平均軌道要素 (角度は度、長半径は AU)。
struct OrbitalElements {
  double semiMajorAxisAu = 0.0;
  double eccentricity = 0.0;
  double inclinationDegrees = 0.0;
  double meanLongitudeDegrees = 0.0;
  double longitudeOfPerihelionDegrees = 0.0;
  double longitudeOfAscendingNodeDegrees = 0.0;
};

// JPL "Keplerian Elements for Approximate Positions of the Major Planets"
// (1800-2050 版) の線形要素。木星・土星には平均経度の長周期補正項が入る。
[[nodiscard]] OrbitalElements planetElements(Planet planet, JulianDate julianDate);

// ケプラー方程式 E - e sin E = M を解く。最大 12 回反復、収束閾値 1e-12 rad。
[[nodiscard]] double solveKeplerEccentricAnomalyRadians(double meanAnomalyRadians,
                                                        double eccentricity);

// 太陽中心の J2000 黄道直交座標 [AU]。
struct CartesianAu {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

[[nodiscard]] CartesianAu heliocentricEcliptic(Planet planet, JulianDate julianDate);

// 地心の見かけの赤道座標。光行時間を反復補正する。
[[nodiscard]] EquatorialCoord planetEquatorial(Planet planet, JulianDate julianDate);

} // namespace astro
