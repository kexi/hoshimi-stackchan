#include "astro/sun.hpp"

#include "astro/angles.hpp"

#include <cmath>

namespace astro {

EclipticCoord sunEcliptic(JulianDate julianDate) {
  const double t = julianDate.centuriesSinceJ2000();

  // 幾何平均黄経 L0 と平均近点角 M (Meeus 25.2 / 25.3)
  const double meanLongitude = normalizeDegrees(280.46646 + 36000.76983 * t + 0.0003032 * t * t);
  const double meanAnomaly = normalizeDegrees(357.52911 + 35999.05029 * t - 0.0001537 * t * t);
  const double meanAnomalyRad = degToRad(meanAnomaly);

  // 中心差 C (Meeus 25.4)
  const double center = (1.914602 - 0.004817 * t - 0.000014 * t * t) * std::sin(meanAnomalyRad) +
                        (0.019993 - 0.000101 * t) * std::sin(2.0 * meanAnomalyRad) +
                        0.000289 * std::sin(3.0 * meanAnomalyRad);

  const double trueLongitude = meanLongitude + center;
  const double trueAnomalyRad = degToRad(meanAnomaly + center);

  // 離心率と動径 R [AU] (Meeus 25.5)
  const double eccentricity = 0.016708634 - 0.000042037 * t - 0.0000001267 * t * t;
  const double radiusAu = (1.000001018 * (1.0 - eccentricity * eccentricity)) /
                          (1.0 + eccentricity * std::cos(trueAnomalyRad));

  // 見かけの黄経: 章動と光行差の補正 (Meeus p.164)。
  // Ω は月の昇交点で、-0.00569 が光行差、-0.00478 sinΩ が章動の主項。
  const double omega = degToRad(normalizeDegrees(125.04 - 1934.136 * t));
  const double apparentLongitude = trueLongitude - 0.00569 - 0.00478 * std::sin(omega);

  EclipticCoord result;
  result.longitudeDegrees = normalizeDegrees(apparentLongitude);
  // 太陽の黄緯は 1.2 秒角を超えず、方位に落とすと無視できる。
  result.latitudeDegrees = 0.0;
  result.distanceAu = radiusAu;
  return result;
}

EquatorialCoord sunEquatorial(JulianDate julianDate) {
  // 見かけの黄経には既に章動が入っているので、変換には真黄道傾斜角を使う。
  return equatorialFromEcliptic(sunEcliptic(julianDate), trueObliquityDegrees(julianDate));
}

} // namespace astro
