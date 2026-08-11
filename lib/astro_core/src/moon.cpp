#include "astro/moon.hpp"

#include "astro/angles.hpp"

#include <array>
#include <cmath>
#include <cstdint>

namespace astro {
namespace {

// Meeus 表 47.A: 黄経 Σl [1e-6 度] と距離 Σr [1e-3 km] の係数。
// 引数の係数は D, M, M', F の順。
struct TermLR {
  std::int8_t d;
  std::int8_t m;
  std::int8_t mPrime;
  std::int8_t f;
  double sinCoefficient; // 黄経
  double cosCoefficient; // 距離
};

constexpr std::array<TermLR, 60> kTermsLR = {{
    {0, 0, 1, 0, 6288774, -20905355},
    {2, 0, -1, 0, 1274027, -3699111},
    {2, 0, 0, 0, 658314, -2955968},
    {0, 0, 2, 0, 213618, -569925},
    {0, 1, 0, 0, -185116, 48888},
    {0, 0, 0, 2, -114332, -3149},
    {2, 0, -2, 0, 58793, 246158},
    {2, -1, -1, 0, 57066, -152138},
    {2, 0, 1, 0, 53322, -170733},
    {2, -1, 0, 0, 45758, -204586},
    {0, 1, -1, 0, -40923, -129620},
    {1, 0, 0, 0, -34720, 108743},
    {0, 1, 1, 0, -30383, 104755},
    {2, 0, 0, -2, 15327, 10321},
    {0, 0, 1, 2, -12528, 0},
    {0, 0, 1, -2, 10980, 79661},
    {4, 0, -1, 0, 10675, -34782},
    {0, 0, 3, 0, 10034, -23210},
    {4, 0, -2, 0, 8548, -21636},
    {2, 1, -1, 0, -7888, 24208},
    {2, 1, 0, 0, -6766, 30824},
    {1, 0, -1, 0, -5163, -8379},
    {1, 1, 0, 0, 4987, -16675},
    {2, -1, 1, 0, 4036, -12831},
    {2, 0, 2, 0, 3994, -10445},
    {4, 0, 0, 0, 3861, -11650},
    {2, 0, -3, 0, 3665, 14403},
    {0, 1, -2, 0, -2689, -7003},
    {2, 0, -1, 2, -2602, 0},
    {2, -1, -2, 0, 2390, 10056},
    {1, 0, 1, 0, -2348, 6322},
    {2, -2, 0, 0, 2236, -9884},
    {0, 1, 2, 0, -2120, 5751},
    {0, 2, 0, 0, -2069, 0},
    {2, -2, -1, 0, 2048, -4950},
    {2, 0, 1, -2, -1773, 4130},
    {2, 0, 0, 2, -1595, 0},
    {4, -1, -1, 0, 1215, -3958},
    {0, 0, 2, 2, -1110, 0},
    {3, 0, -1, 0, -892, 3258},
    {2, 1, 1, 0, -810, 2616},
    {4, -1, -2, 0, 759, -1897},
    {0, 2, -1, 0, -713, -2117},
    {2, 2, -1, 0, -700, 2354},
    {2, 1, -2, 0, 691, 0},
    {2, -1, 0, -2, 596, 0},
    {4, 0, 1, 0, 549, -1423},
    {0, 0, 4, 0, 537, -1117},
    {4, -1, 0, 0, 520, -1571},
    {1, 0, -2, 0, -487, -1739},
    {2, 1, 0, -2, -399, 0},
    {0, 0, 2, -2, -381, -4421},
    {1, 1, 1, 0, 351, 0},
    {3, 0, -2, 0, -340, 0},
    {4, 0, -3, 0, 330, 0},
    {2, -1, 2, 0, 327, 0},
    {0, 2, 1, 0, -323, 1165},
    {1, 1, -1, 0, 299, 0},
    {2, 0, 3, 0, 294, 0},
    {2, 0, -1, -2, 0, 8752},
}};

// Meeus 表 47.B: 黄緯 Σb [1e-6 度]。
struct TermB {
  std::int8_t d;
  std::int8_t m;
  std::int8_t mPrime;
  std::int8_t f;
  double sinCoefficient;
};

constexpr std::array<TermB, 60> kTermsB = {{
    {0, 0, 0, 1, 5128122}, {0, 0, 1, 1, 280602},  {0, 0, 1, -1, 277693}, {2, 0, 0, -1, 173237},
    {2, 0, -1, 1, 55413},  {2, 0, -1, -1, 46271}, {2, 0, 0, 1, 32573},   {0, 0, 2, 1, 17198},
    {2, 0, 1, -1, 9266},   {0, 0, 2, -1, 8822},   {2, -1, 0, -1, 8216},  {2, 0, -2, -1, 4324},
    {2, 0, 1, 1, 4200},    {2, 1, 0, -1, -3359},  {2, -1, -1, 1, 2463},  {2, -1, 0, 1, 2211},
    {2, -1, -1, -1, 2065}, {0, 1, -1, -1, -1870}, {4, 0, -1, -1, 1828},  {0, 1, 0, 1, -1794},
    {0, 0, 0, 3, -1749},   {0, 1, -1, 1, -1565},  {1, 0, 0, 1, -1491},   {0, 1, 1, 1, -1475},
    {0, 1, 1, -1, -1410},  {0, 1, 0, -1, -1344},  {1, 0, 0, -1, -1335},  {0, 0, 3, 1, 1107},
    {4, 0, 0, -1, 1021},   {4, 0, -1, 1, 833},    {0, 0, 1, -3, 777},    {4, 0, -2, 1, 671},
    {2, 0, 0, -3, 607},    {2, 0, 2, -1, 596},    {2, -1, 1, -1, 491},   {2, 0, -2, 1, -451},
    {0, 0, 3, -1, 439},    {2, 0, 2, 1, 422},     {2, 0, -3, -1, 421},   {2, 1, -1, 1, -366},
    {2, 1, 0, 1, -351},    {4, 0, 0, 1, 331},     {2, -1, 1, 1, 315},    {2, -2, 0, -1, 302},
    {0, 0, 1, 3, -283},    {2, 1, 1, -1, -229},   {1, 1, 0, -1, 223},    {1, 1, 0, 1, 223},
    {0, 1, -2, -1, -220},  {2, 1, -1, -1, -220},  {1, 0, 1, 1, -185},    {2, -1, -2, -1, 181},
    {0, 1, 2, 1, -177},    {4, 0, -2, -1, 176},   {4, -1, -1, -1, 166},  {1, 0, 1, -1, -164},
    {4, 0, 1, -1, 132},    {1, 0, -1, -1, -119},  {4, -1, 0, -1, 115},   {2, -2, 0, 1, 107},
}};

} // namespace

EclipticCoord moonEcliptic(JulianDate julianDate) {
  const double t = julianDate.centuriesSinceJ2000();

  // 基本引数 (Meeus 47.1-47.5)
  const double moonMeanLongitude =
      normalizeDegrees(218.3164477 + 481267.88123421 * t - 0.0015786 * t * t +
                       t * t * t / 538841.0 - t * t * t * t / 65194000.0);
  const double meanElongation =
      normalizeDegrees(297.8501921 + 445267.1114034 * t - 0.0018819 * t * t + t * t * t / 545868.0 -
                       t * t * t * t / 113065000.0);
  const double sunMeanAnomaly = normalizeDegrees(357.5291092 + 35999.0502909 * t -
                                                 0.0001536 * t * t + t * t * t / 24490000.0);
  const double moonMeanAnomaly =
      normalizeDegrees(134.9633964 + 477198.8675055 * t + 0.0087414 * t * t + t * t * t / 69699.0 -
                       t * t * t * t / 14712000.0);
  const double argumentOfLatitude =
      normalizeDegrees(93.272095 + 483202.0175233 * t - 0.0036539 * t * t - t * t * t / 3526000.0 +
                       t * t * t * t / 863310000.0);

  // 金星・木星・地球扁平による加算項 (Meeus p.338)
  const double a1 = normalizeDegrees(119.75 + 131.849 * t);
  const double a2 = normalizeDegrees(53.09 + 479264.290 * t);
  const double a3 = normalizeDegrees(313.45 + 481266.484 * t);

  // 離心率補正 E: 太陽の平均近点角を含む項に E (M が ±2 なら E^2) を掛ける
  const double eccentricityFactor = 1.0 - 0.002516 * t - 0.0000074 * t * t;

  const double dRad = degToRad(meanElongation);
  const double mRad = degToRad(sunMeanAnomaly);
  const double mPrimeRad = degToRad(moonMeanAnomaly);
  const double fRad = degToRad(argumentOfLatitude);

  double sumLongitude = 0.0;
  double sumRadius = 0.0;
  for (const TermLR& term : kTermsLR) {
    const double argument = term.d * dRad + term.m * mRad + term.mPrime * mPrimeRad + term.f * fRad;
    double eccentricityScale = 1.0;
    if (term.m == 1 || term.m == -1) {
      eccentricityScale = eccentricityFactor;
    } else if (term.m == 2 || term.m == -2) {
      eccentricityScale = eccentricityFactor * eccentricityFactor;
    }
    sumLongitude += term.sinCoefficient * eccentricityScale * std::sin(argument);
    sumRadius += term.cosCoefficient * eccentricityScale * std::cos(argument);
  }

  double sumLatitude = 0.0;
  for (const TermB& term : kTermsB) {
    const double argument = term.d * dRad + term.m * mRad + term.mPrime * mPrimeRad + term.f * fRad;
    double eccentricityScale = 1.0;
    if (term.m == 1 || term.m == -1) {
      eccentricityScale = eccentricityFactor;
    } else if (term.m == 2 || term.m == -2) {
      eccentricityScale = eccentricityFactor * eccentricityFactor;
    }
    sumLatitude += term.sinCoefficient * eccentricityScale * std::sin(argument);
  }

  // 加算項 (Meeus p.338)
  sumLongitude += 3958.0 * std::sin(degToRad(a1)) +
                  1962.0 * std::sin(degToRad(moonMeanLongitude - argumentOfLatitude)) +
                  318.0 * std::sin(degToRad(a2));
  sumLatitude += -2235.0 * std::sin(degToRad(moonMeanLongitude)) + 382.0 * std::sin(degToRad(a3)) +
                 175.0 * std::sin(degToRad(a1 - argumentOfLatitude)) +
                 175.0 * std::sin(degToRad(a1 + argumentOfLatitude)) +
                 127.0 * std::sin(degToRad(moonMeanLongitude - moonMeanAnomaly)) -
                 115.0 * std::sin(degToRad(moonMeanLongitude + moonMeanAnomaly));

  // Σl, Σb は 1e-6 度、Σr は 1e-3 km
  const double geocentricLongitude = moonMeanLongitude + sumLongitude / 1000000.0;
  const double geocentricLatitude = sumLatitude / 1000000.0;
  const double distanceKm = 385000.56 + sumRadius / 1000.0;

  // 視黄経にするため章動 Δψ を足す (Meeus p.342)
  const double apparentLongitude = geocentricLongitude + nutation(julianDate).longitudeDegrees;

  constexpr double kKilometersPerAu = 149597870.7;

  EclipticCoord result;
  result.longitudeDegrees = normalizeDegrees(apparentLongitude);
  result.latitudeDegrees = geocentricLatitude;
  result.distanceAu = distanceKm / kKilometersPerAu;
  return result;
}

EquatorialCoord moonEquatorial(JulianDate julianDate) {
  return equatorialFromEcliptic(moonEcliptic(julianDate), trueObliquityDegrees(julianDate));
}

} // namespace astro
