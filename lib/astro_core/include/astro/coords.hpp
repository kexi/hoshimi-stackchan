#pragma once

namespace astro {

// 見かけの赤道座標。distanceAu が 0 なら距離未知 (視差補正を行わない)。
struct EquatorialCoord {
  double rightAscensionDegrees = 0.0; // [0,360)
  double declinationDegrees = 0.0;    // [-90,90]
  double distanceAu = 0.0;
};

struct EclipticCoord {
  double longitudeDegrees = 0.0;
  double latitudeDegrees = 0.0;
  double distanceAu = 0.0;
};

// 地平座標。方位角は真北 0、東 90 の時計回り (航法の慣習)。
struct HorizontalCoord {
  double azimuthDegrees = 0.0;
  double altitudeDegrees = 0.0;
};

struct Observer {
  double latitudeDegrees = 0.0;
  double longitudeEastDegrees = 0.0;
  double elevationMeters = 0.0;
};

[[nodiscard]] EquatorialCoord equatorialFromEcliptic(EclipticCoord ecliptic,
                                                     double obliquityDegrees);

[[nodiscard]] EclipticCoord eclipticFromEquatorial(EquatorialCoord equatorial,
                                                   double obliquityDegrees);

// 赤道座標 → 地平座標。地方視恒星時から時角を作る。大気差は含まない。
[[nodiscard]] HorizontalCoord horizontalFromEquatorial(EquatorialCoord equatorial,
                                                       Observer observer,
                                                       double localApparentSiderealTimeDeg);

// 地心 → 測心 (周日視差)。月は最大 1 度ずれるので必須。
// distanceAu <= 0 のときは入力をそのまま返す。
[[nodiscard]] EquatorialCoord applyTopocentricParallax(EquatorialCoord geocentric,
                                                       Observer observer,
                                                       double localApparentSiderealTimeDeg);

// 大気差 [度]。Bennett の式。地平線下 (-1 度未満) では 0 を返す。
[[nodiscard]] double atmosphericRefractionDegrees(double trueAltitudeDegrees);

} // namespace astro
