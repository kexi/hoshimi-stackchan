#include "astro/planets.hpp"

#include "astro/angles.hpp"
#include "astro/sun.hpp"

#include <cmath>

namespace astro {
namespace {

// JPL の近似要素表。値は元期 J2000 の要素と、ユリウス世紀あたりの変化率。
struct ElementSet {
  double semiMajorAxisAu;
  double semiMajorAxisRate;
  double eccentricity;
  double eccentricityRate;
  double inclinationDegrees;
  double inclinationRate;
  double meanLongitudeDegrees;
  double meanLongitudeRate;
  double longitudeOfPerihelionDegrees;
  double longitudeOfPerihelionRate;
  double longitudeOfAscendingNodeDegrees;
  double longitudeOfAscendingNodeRate;
  // 木星以遠のみ使う平均近点角の補正項 (JPL 表 2)。使わない天体では 0。
  double b;
  double c;
  double s;
  double f;
};

// 1800-2050 年で有効な要素 (JPL 表 1 + 表 2)
constexpr ElementSet kMercury = {0.38709927,  0.00000037,  0.20563593,   0.00001906,
                                 7.00497902,  -0.00594749, 252.25032350, 149472.67411175,
                                 77.45779628, 0.16047689,  48.33076593,  -0.12534081,
                                 0.0,         0.0,         0.0,          0.0};
constexpr ElementSet kVenus = {0.72333566,   0.00000390,  0.00677672,   -0.00004107,
                               3.39467605,   -0.00078890, 181.97909950, 58517.81538729,
                               131.60246718, 0.00268329,  76.67984255,  -0.27769418,
                               0.0,          0.0,         0.0,          0.0};
constexpr ElementSet kEarth = {1.00000261,   0.00000562,  0.01671123,   -0.00004392,
                               -0.00001531,  -0.01294668, 100.46457166, 35999.37244981,
                               102.93768193, 0.32327364,  0.0,          0.0,
                               0.0,          0.0,         0.0,          0.0};
constexpr ElementSet kMars = {1.52371034,   0.00001847,  0.09339410,  0.00007882,
                              1.84969142,   -0.00813131, -4.55343205, 19140.30268499,
                              -23.94362959, 0.44441088,  49.55953891, -0.29257343,
                              0.0,          0.0,         0.0,         0.0};
constexpr ElementSet kJupiter = {5.20288700,  -0.00011607, 0.04838624,   -0.00013253,
                                 1.30439695,  -0.00183714, 34.39644051,  3034.74612775,
                                 14.72847983, 0.21252668,  100.47390909, 0.20469106,
                                 -0.00012452, 0.06064060,  -0.35635438,  38.35125000};
constexpr ElementSet kSaturn = {9.53667594,  -0.00125060, 0.05386179,   -0.00050991,
                                2.48599187,  0.00193609,  49.95424423,  1222.49362201,
                                92.59887831, -0.41897216, 113.66242448, -0.28867794,
                                0.00025899,  -0.13434469, 0.87320147,   38.35125000};

const ElementSet& elementSetFor(Planet planet) {
  switch (planet) {
  case Planet::Mercury:
    return kMercury;
  case Planet::Venus:
    return kVenus;
  case Planet::Earth:
    return kEarth;
  case Planet::Mars:
    return kMars;
  case Planet::Jupiter:
    return kJupiter;
  case Planet::Saturn:
    break;
  }
  return kSaturn;
}

// 光速で 1 AU を進む時間 [日]
constexpr double kLightTimePerAuDays = 0.005775518331;

CartesianAu heliocentricFromElements(const ElementSet& set, double t) {
  const double semiMajorAxis = set.semiMajorAxisAu + set.semiMajorAxisRate * t;
  const double eccentricity = set.eccentricity + set.eccentricityRate * t;
  const double inclination = set.inclinationDegrees + set.inclinationRate * t;
  const double meanLongitude = set.meanLongitudeDegrees + set.meanLongitudeRate * t;
  const double longitudeOfPerihelion =
      set.longitudeOfPerihelionDegrees + set.longitudeOfPerihelionRate * t;
  const double longitudeOfNode =
      set.longitudeOfAscendingNodeDegrees + set.longitudeOfAscendingNodeRate * t;

  const double argumentOfPerihelion = longitudeOfPerihelion - longitudeOfNode;

  // 木星・土星は線形項だけでは 1800-2050 でずれるので、JPL 表 2 の補正を平均近点角に足す。
  double meanAnomaly = meanLongitude - longitudeOfPerihelion;
  meanAnomaly +=
      set.b * t * t + set.c * std::cos(degToRad(set.f * t)) + set.s * std::sin(degToRad(set.f * t));
  meanAnomaly = normalizeSignedDegrees(meanAnomaly);

  const double eccentricAnomaly =
      solveKeplerEccentricAnomalyRadians(degToRad(meanAnomaly), eccentricity);

  // 軌道面内の座標
  const double xOrbital = semiMajorAxis * (std::cos(eccentricAnomaly) - eccentricity);
  const double yOrbital =
      semiMajorAxis * std::sqrt(1.0 - eccentricity * eccentricity) * std::sin(eccentricAnomaly);

  // 近日点引数 → 昇交点黄経 → 軌道傾斜 の順に回して黄道座標へ
  const double cosArgument = std::cos(degToRad(argumentOfPerihelion));
  const double sinArgument = std::sin(degToRad(argumentOfPerihelion));
  const double cosNode = std::cos(degToRad(longitudeOfNode));
  const double sinNode = std::sin(degToRad(longitudeOfNode));
  const double cosInclination = std::cos(degToRad(inclination));
  const double sinInclination = std::sin(degToRad(inclination));

  CartesianAu result;
  result.x = (cosArgument * cosNode - sinArgument * sinNode * cosInclination) * xOrbital +
             (-sinArgument * cosNode - cosArgument * sinNode * cosInclination) * yOrbital;
  result.y = (cosArgument * sinNode + sinArgument * cosNode * cosInclination) * xOrbital +
             (-sinArgument * sinNode + cosArgument * cosNode * cosInclination) * yOrbital;
  result.z = sinArgument * sinInclination * xOrbital + cosArgument * sinInclination * yOrbital;
  return result;
}

} // namespace

OrbitalElements planetElements(Planet planet, JulianDate julianDate) {
  const ElementSet& set = elementSetFor(planet);
  const double t = julianDate.centuriesSinceJ2000();

  OrbitalElements elements;
  elements.semiMajorAxisAu = set.semiMajorAxisAu + set.semiMajorAxisRate * t;
  elements.eccentricity = set.eccentricity + set.eccentricityRate * t;
  elements.inclinationDegrees = set.inclinationDegrees + set.inclinationRate * t;
  elements.meanLongitudeDegrees =
      normalizeDegrees(set.meanLongitudeDegrees + set.meanLongitudeRate * t);
  elements.longitudeOfPerihelionDegrees =
      normalizeDegrees(set.longitudeOfPerihelionDegrees + set.longitudeOfPerihelionRate * t);
  elements.longitudeOfAscendingNodeDegrees =
      normalizeDegrees(set.longitudeOfAscendingNodeDegrees + set.longitudeOfAscendingNodeRate * t);
  return elements;
}

double solveKeplerEccentricAnomalyRadians(double meanAnomalyRadians, double eccentricity) {
  // Newton 法。惑星の離心率 (最大 0.21) では初期値 M で十分速く収束する。
  double eccentricAnomaly = meanAnomalyRadians;
  for (int iteration = 0; iteration < 12; ++iteration) {
    const double residual =
        eccentricAnomaly - eccentricity * std::sin(eccentricAnomaly) - meanAnomalyRadians;
    const double derivative = 1.0 - eccentricity * std::cos(eccentricAnomaly);
    const double delta = residual / derivative;
    eccentricAnomaly -= delta;
    if (std::fabs(delta) < 1e-12) {
      break;
    }
  }
  return eccentricAnomaly;
}

CartesianAu heliocentricEcliptic(Planet planet, JulianDate julianDate) {
  return heliocentricFromElements(elementSetFor(planet), julianDate.centuriesSinceJ2000());
}

EquatorialCoord planetEquatorial(Planet planet, JulianDate julianDate) {
  const double t = julianDate.centuriesSinceJ2000();
  const CartesianAu earth = heliocentricFromElements(kEarth, t);

  // 光行時間の補正: 惑星の位置は「光が出た時刻」のものを使う。
  // 距離が分かってからでないと時刻が決まらないので反復する (2 回で 1e-8 AU に収まる)。
  CartesianAu planetPosition = heliocentricFromElements(elementSetFor(planet), t);
  double distanceAu = 0.0;
  for (int iteration = 0; iteration < 3; ++iteration) {
    const double dx = planetPosition.x - earth.x;
    const double dy = planetPosition.y - earth.y;
    const double dz = planetPosition.z - earth.z;
    distanceAu = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double lightTimeCenturies = (distanceAu * kLightTimePerAuDays) / 36525.0;
    planetPosition = heliocentricFromElements(elementSetFor(planet), t - lightTimeCenturies);
  }

  // 地心黄道座標
  const double dx = planetPosition.x - earth.x;
  const double dy = planetPosition.y - earth.y;
  const double dz = planetPosition.z - earth.z;

  EclipticCoord ecliptic;
  ecliptic.longitudeDegrees = normalizeDegrees(radToDeg(std::atan2(dy, dx)));
  ecliptic.latitudeDegrees = radToDeg(std::atan2(dz, std::sqrt(dx * dx + dy * dy)));
  ecliptic.distanceAu = distanceAu;

  // 年周光行差 (Meeus 23.2)。地球の公転による見かけのずれで最大 20.5 秒角 = 0.0057 度。
  // 章動と同程度の大きさなので、片方だけ入れるのは中途半端になる。
  constexpr double kAberrationConstantDegrees = 20.49552 / 3600.0;
  const double sunLongitude = sunEcliptic(julianDate).longitudeDegrees;
  const double eccentricity = 0.016708634 - 0.000042037 * t;
  const double perihelionLongitude = 102.93735 + 1.71946 * t;
  const double lambda = ecliptic.longitudeDegrees;
  const double beta = ecliptic.latitudeDegrees;

  const double deltaLongitude =
      (-kAberrationConstantDegrees * std::cos(degToRad(sunLongitude - lambda)) +
       eccentricity * kAberrationConstantDegrees *
           std::cos(degToRad(perihelionLongitude - lambda))) /
      std::cos(degToRad(beta));
  const double deltaLatitude = -kAberrationConstantDegrees * std::sin(degToRad(beta)) *
                               (std::sin(degToRad(sunLongitude - lambda)) -
                                eccentricity * std::sin(degToRad(perihelionLongitude - lambda)));

  ecliptic.longitudeDegrees = normalizeDegrees(lambda + deltaLongitude);
  ecliptic.latitudeDegrees = beta + deltaLatitude;

  // 章動 Δψ を足して視黄経にしてから、真黄道傾斜角で赤道座標へ移す。
  ecliptic.longitudeDegrees =
      normalizeDegrees(ecliptic.longitudeDegrees + nutation(julianDate).longitudeDegrees);

  return equatorialFromEcliptic(ecliptic, trueObliquityDegrees(julianDate));
}

} // namespace astro
