#include "astro/coords.hpp"

#include "astro/angles.hpp"

#include <cmath>

namespace astro {
namespace {

// 地球の赤道半径に対する天文単位。視差計算で使う。
constexpr double kEarthRadiiPerAu = 23454.7925;
constexpr double kEarthFlattening = 1.0 / 298.257223563;
constexpr double kEarthEquatorialRadiusMeters = 6378137.0;

// 観測者の地心距離を、赤道半径を単位として (rho*sin(phi'), rho*cos(phi')) で返す。
// Meeus ch.11。地球の扁平を考慮する。
struct GeocentricObserver {
  double rhoSinPhiPrime = 0.0;
  double rhoCosPhiPrime = 0.0;
};

GeocentricObserver geocentricObserver(const Observer& observer) {
  const double phi = degToRad(observer.latitudeDegrees);
  const double u = std::atan((1.0 - kEarthFlattening) * std::tan(phi));
  const double elevationInRadii = observer.elevationMeters / kEarthEquatorialRadiusMeters;

  GeocentricObserver result;
  result.rhoSinPhiPrime = (1.0 - kEarthFlattening) * std::sin(u) + elevationInRadii * std::sin(phi);
  result.rhoCosPhiPrime = std::cos(u) + elevationInRadii * std::cos(phi);
  return result;
}

} // namespace

EquatorialCoord equatorialFromEcliptic(EclipticCoord ecliptic, double obliquityDegrees) {
  const double lambda = degToRad(ecliptic.longitudeDegrees);
  const double beta = degToRad(ecliptic.latitudeDegrees);
  const double eps = degToRad(obliquityDegrees);

  const double sinDec =
      std::sin(beta) * std::cos(eps) + std::cos(beta) * std::sin(eps) * std::sin(lambda);
  const double y = std::sin(lambda) * std::cos(eps) - std::tan(beta) * std::sin(eps);
  const double x = std::cos(lambda);

  EquatorialCoord result;
  result.rightAscensionDegrees = normalizeDegrees(radToDeg(std::atan2(y, x)));
  result.declinationDegrees = radToDeg(std::asin(sinDec));
  result.distanceAu = ecliptic.distanceAu;
  return result;
}

EclipticCoord eclipticFromEquatorial(EquatorialCoord equatorial, double obliquityDegrees) {
  const double ra = degToRad(equatorial.rightAscensionDegrees);
  const double dec = degToRad(equatorial.declinationDegrees);
  const double eps = degToRad(obliquityDegrees);

  const double sinBeta =
      std::sin(dec) * std::cos(eps) - std::cos(dec) * std::sin(eps) * std::sin(ra);
  const double y = std::sin(ra) * std::cos(eps) + std::tan(dec) * std::sin(eps);
  const double x = std::cos(ra);

  EclipticCoord result;
  result.longitudeDegrees = normalizeDegrees(radToDeg(std::atan2(y, x)));
  result.latitudeDegrees = radToDeg(std::asin(sinBeta));
  result.distanceAu = equatorial.distanceAu;
  return result;
}

HorizontalCoord horizontalFromEquatorial(EquatorialCoord equatorial, Observer observer,
                                         double localApparentSiderealTimeDeg) {
  // 時角 H = 地方視恒星時 - 赤経
  const double hourAngle =
      degToRad(normalizeDegrees(localApparentSiderealTimeDeg - equatorial.rightAscensionDegrees));
  const double dec = degToRad(equatorial.declinationDegrees);
  const double phi = degToRad(observer.latitudeDegrees);

  const double sinAlt =
      std::sin(phi) * std::sin(dec) + std::cos(phi) * std::cos(dec) * std::cos(hourAngle);

  // Meeus は方位角を南basis で定義するが、ここでは北 0・東 90 に直して返す。
  // atan2 の分子を -sin(H) にすることで、南basis の式に 180 度足したのと同じになる。
  const double y = std::sin(hourAngle);
  const double x = std::cos(hourAngle) * std::sin(phi) - std::tan(dec) * std::cos(phi);
  const double azimuthSouthBased = radToDeg(std::atan2(y, x));

  HorizontalCoord result;
  result.azimuthDegrees = normalizeDegrees(azimuthSouthBased + 180.0);
  result.altitudeDegrees = radToDeg(std::asin(sinAlt));
  return result;
}

EquatorialCoord applyTopocentricParallax(EquatorialCoord geocentric, Observer observer,
                                         double localApparentSiderealTimeDeg) {
  if (!(geocentric.distanceAu > 0.0)) {
    return geocentric;
  }

  const GeocentricObserver site = geocentricObserver(observer);
  // 赤道地平視差 sin(pi) = (地球赤道半径) / (距離)
  const double sinParallax = 1.0 / (kEarthRadiiPerAu * geocentric.distanceAu);

  const double hourAngle =
      degToRad(normalizeDegrees(localApparentSiderealTimeDeg - geocentric.rightAscensionDegrees));
  const double dec = degToRad(geocentric.declinationDegrees);

  // Meeus (40.2)/(40.3)
  const double deltaRaNumerator = -site.rhoCosPhiPrime * sinParallax * std::sin(hourAngle);
  const double deltaRaDenominator =
      std::cos(dec) - site.rhoCosPhiPrime * sinParallax * std::cos(hourAngle);
  const double deltaRa = std::atan2(deltaRaNumerator, deltaRaDenominator);

  const double topocentricDec = std::atan2(
      (std::sin(dec) - site.rhoSinPhiPrime * sinParallax) * std::cos(deltaRa), deltaRaDenominator);

  EquatorialCoord result;
  result.rightAscensionDegrees =
      normalizeDegrees(geocentric.rightAscensionDegrees + radToDeg(deltaRa));
  result.declinationDegrees = radToDeg(topocentricDec);
  result.distanceAu = geocentric.distanceAu;
  return result;
}

double atmosphericRefractionDegrees(double trueAltitudeDegrees) {
  // 地平線よりかなり下では大気差モデルが破綻するので補正しない。
  if (trueAltitudeDegrees < -1.0) {
    return 0.0;
  }
  // Bennett の式 (Meeus 16.4)。真高度 → 見かけの高度への差分を分角で返す。
  const double denominatorDeg = trueAltitudeDegrees + 7.31 / (trueAltitudeDegrees + 4.4);
  const double refractionArcmin = 1.02 / std::tan(degToRad(denominatorDeg));
  return refractionArcmin / 60.0;
}

} // namespace astro
