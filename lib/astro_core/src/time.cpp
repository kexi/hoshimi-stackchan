#include "astro/time.hpp"

#include "astro/angles.hpp"

#include <cmath>

namespace astro {
namespace {

// Unix epoch (1970-01-01T00:00:00Z) のユリウス日
constexpr double kUnixEpochJd = 2440587.5;
constexpr double kJ2000Jd = 2451545.0;
constexpr double kSecondsPerDay = 86400.0;

} // namespace

double JulianDate::centuriesSinceJ2000() const { return (jd - kJ2000Jd) / 36525.0; }

double JulianDate::daysSinceJ2000() const { return jd - kJ2000Jd; }

JulianDate julianDateFromUnixSeconds(std::int64_t unixSeconds) {
  return JulianDate{kUnixEpochJd + static_cast<double>(unixSeconds) / kSecondsPerDay};
}

std::int64_t unixSecondsFromJulianDate(JulianDate julianDate) {
  return static_cast<std::int64_t>(std::llround((julianDate.jd - kUnixEpochJd) * kSecondsPerDay));
}

double greenwichMeanSiderealTimeDegrees(JulianDate julianDate) {
  const double d = julianDate.daysSinceJ2000();
  const double t = julianDate.centuriesSinceJ2000();
  // Meeus (12.4): 任意の時刻に対する GMST を度で直接与える式。
  const double theta =
      280.46061837 + 360.98564736629 * d + 0.000387933 * t * t - (t * t * t) / 38710000.0;
  return normalizeDegrees(theta);
}

double localMeanSiderealTimeDegrees(JulianDate julianDate, double longitudeEastDegrees) {
  return normalizeDegrees(greenwichMeanSiderealTimeDegrees(julianDate) + longitudeEastDegrees);
}

Nutation nutation(JulianDate julianDate) {
  const double t = julianDate.centuriesSinceJ2000();

  // 月の昇交点の平均黄経 Ω
  const double omega = degToRad(normalizeDegrees(125.04452 - 1934.136261 * t));
  // 太陽の平均黄経 L、月の平均黄経 L'
  const double sunLongitude = degToRad(normalizeDegrees(280.4665 + 36000.7698 * t));
  const double moonLongitude = degToRad(normalizeDegrees(218.3165 + 481267.8813 * t));

  // Meeus ch.22 の主要 4 項 (単位 0.0001 秒角 → 度)
  const double deltaPsiArcsec = -17.20 * std::sin(omega) - 1.32 * std::sin(2.0 * sunLongitude) -
                                0.23 * std::sin(2.0 * moonLongitude) + 0.21 * std::sin(2.0 * omega);
  const double deltaEpsArcsec = 9.20 * std::cos(omega) + 0.57 * std::cos(2.0 * sunLongitude) +
                                0.10 * std::cos(2.0 * moonLongitude) - 0.09 * std::cos(2.0 * omega);

  return Nutation{deltaPsiArcsec / 3600.0, deltaEpsArcsec / 3600.0};
}

double meanObliquityDegrees(JulianDate julianDate) {
  // Meeus (22.2) は U = T/100 の 10 次多項式。1000 年スケールで数秒角に収まる。
  const double u = julianDate.centuriesSinceJ2000() / 100.0;
  const double arcsec =
      84381.448 - 4680.93 * u - 1.55 * u * u + 1999.25 * u * u * u - 51.38 * u * u * u * u -
      249.67 * u * u * u * u * u - 39.05 * u * u * u * u * u * u +
      7.12 * u * u * u * u * u * u * u + 27.87 * u * u * u * u * u * u * u * u +
      5.79 * u * u * u * u * u * u * u * u * u + 2.45 * u * u * u * u * u * u * u * u * u * u;
  return arcsec / 3600.0;
}

double trueObliquityDegrees(JulianDate julianDate) {
  return meanObliquityDegrees(julianDate) + nutation(julianDate).obliquityDegrees;
}

double localApparentSiderealTimeDegrees(JulianDate julianDate, double longitudeEastDegrees) {
  const Nutation nut = nutation(julianDate);
  // 分点の式: Δψ cos ε を恒星時に足す。
  const double equationOfEquinoxes =
      nut.longitudeDegrees * std::cos(degToRad(trueObliquityDegrees(julianDate)));
  return normalizeDegrees(localMeanSiderealTimeDegrees(julianDate, longitudeEastDegrees) +
                          equationOfEquinoxes);
}

} // namespace astro
