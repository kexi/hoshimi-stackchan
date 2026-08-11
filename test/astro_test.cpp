// astro_core が保証すること:
//  - 角度の正規化が境界 (0, 360, 負値) で破綻しない
//  - Unix 秒 ↔ ユリウス日が既知の対応と一致し、往復で戻る
//  - 恒星時・黄道傾斜角が Meeus の例題と一致する
//  - 赤道 ↔ 黄道の変換が往復不変で、地平座標が Meeus 例題 13.b と一致する
//  - 周日視差が月に対して有意に効き、遠方天体には効かない
//  - 極地・日付変更線で NaN を出さない

#include "astro/angles.hpp"
#include "astro/coords.hpp"
#include "astro/ephemeris.hpp"
#include "astro/moon.hpp"
#include "astro/planets.hpp"
#include "astro/sun.hpp"
#include "astro/time.hpp"

#include "test_support.hpp"

#include <cmath>

namespace {

// 2 つの赤道座標の大円距離 [度]。RA の差は cos(δ) で縮むので、
// 単純な成分差ではなく実際に空の上でどれだけ離れているかで測る。
double angularSeparationDegrees(const astro::EquatorialCoord& a, const astro::EquatorialCoord& b) {
  const double deltaRa = astro::angularDifference(b.rightAscensionDegrees, a.rightAscensionDegrees);
  const double meanDec = astro::degToRad(0.5 * (a.declinationDegrees + b.declinationDegrees));
  return std::hypot(deltaRa * std::cos(meanDec), a.declinationDegrees - b.declinationDegrees);
}

void testAngleNormalization() {
  CHECK_NEAR(astro::normalizeDegrees(0.0), 0.0, 1e-12);
  CHECK_NEAR(astro::normalizeDegrees(360.0), 0.0, 1e-12);
  CHECK_NEAR(astro::normalizeDegrees(-0.0), 0.0, 1e-12);
  CHECK_NEAR(astro::normalizeDegrees(370.5), 10.5, 1e-12);
  CHECK_NEAR(astro::normalizeDegrees(-10.0), 350.0, 1e-12);
  CHECK_NEAR(astro::normalizeDegrees(-720.5), 359.5, 1e-12);

  // 極小の負値を fmod に食わせても [0,360) を守る
  CHECK_TRUE(astro::normalizeDegrees(-1e-18) >= 0.0);
  CHECK_TRUE(astro::normalizeDegrees(-1e-18) < 360.0);

  CHECK_NEAR(astro::normalizeSignedDegrees(180.0), 180.0, 1e-12);
  CHECK_NEAR(astro::normalizeSignedDegrees(181.0), -179.0, 1e-12);
  CHECK_NEAR(astro::normalizeSignedDegrees(-181.0), 179.0, 1e-12);

  // 359 度から 1 度へは +2 度であって -358 度ではない
  CHECK_NEAR(astro::angularDifference(359.0, 1.0), 2.0, 1e-12);
  CHECK_NEAR(astro::angularDifference(1.0, 359.0), -2.0, 1e-12);
}

void testJulianDate() {
  // J2000.0 元期: 2000-01-01T12:00:00Z = JD 2451545.0
  const std::int64_t j2000Unix = 946728000;
  const astro::JulianDate j2000 = astro::julianDateFromUnixSeconds(j2000Unix);
  CHECK_NEAR(j2000.jd, 2451545.0, 1e-9);
  CHECK_NEAR(j2000.daysSinceJ2000(), 0.0, 1e-9);
  CHECK_NEAR(j2000.centuriesSinceJ2000(), 0.0, 1e-12);

  // Unix epoch 自体
  CHECK_NEAR(astro::julianDateFromUnixSeconds(0).jd, 2440587.5, 1e-9);

  // 往復で戻る
  const std::int64_t sample = 1786000000;
  CHECK_TRUE(astro::unixSecondsFromJulianDate(astro::julianDateFromUnixSeconds(sample)) == sample);
}

void testSiderealTime() {
  // Meeus 例題 12.a: 1987-04-10 0h UT → GMST = 13h10m46.3668s
  // = 13.179546333h * 15 = 197.693195 度
  const astro::JulianDate example12a{2446895.5};
  CHECK_NEAR_ANGLE(astro::greenwichMeanSiderealTimeDegrees(example12a), 197.693195, 1e-4);

  // Meeus 例題 12.b: 1987-04-10 19h21m00s UT → GMST = 8h34m57.0896s
  // = 8.582524889h * 15 = 128.737873 度
  const astro::JulianDate example12b{2446896.30625};
  CHECK_NEAR_ANGLE(astro::greenwichMeanSiderealTimeDegrees(example12b), 128.737873, 1e-4);

  // 東経は恒星時に素直に足される
  const double greenwich = astro::greenwichMeanSiderealTimeDegrees(example12b);
  CHECK_NEAR_ANGLE(astro::localMeanSiderealTimeDegrees(example12b, 139.7),
                   astro::normalizeDegrees(greenwich + 139.7), 1e-9);

  // 分点の式 (Δψ cos ε) は最大でも数秒角。方位精度 0.1 度の要求に対しては無視できる
  // 大きさであることを、上限を押さえる形で保証する。
  const double apparent = astro::localApparentSiderealTimeDegrees(example12b, 0.0);
  const double equationOfEquinoxesArcsec =
      std::fabs(astro::angularDifference(greenwich, apparent)) * 3600.0;
  CHECK_TRUE(equationOfEquinoxesArcsec > 0.0); // 補正が実際に効いている
  CHECK_TRUE(equationOfEquinoxesArcsec < 20.0);
}

void testObliquity() {
  // Meeus 例題 22.a: 1987-04-10 0h TD で ε0 = 23°26'27.407" = 23.4409464 度
  const astro::JulianDate example22a{2446895.5};
  CHECK_NEAR(astro::meanObliquityDegrees(example22a), 23.4409464, 1e-6);

  // 同例題の章動: Δψ = -3.788", Δε = +9.443"
  const astro::Nutation nut = astro::nutation(example22a);
  CHECK_NEAR(nut.longitudeDegrees * 3600.0, -3.788, 0.5);
  CHECK_NEAR(nut.obliquityDegrees * 3600.0, 9.443, 0.5);

  // 真傾斜角 ε = 23°26'36.850"
  CHECK_NEAR(astro::trueObliquityDegrees(example22a), 23.4435694, 2e-4);
}

void testEclipticEquatorialRoundTrip() {
  // Meeus 例題 13.a: λ = 113.215630 度, β = +6.684170 度, ε = 23.4392911 度
  const double obliquity = 23.4392911;
  const astro::EclipticCoord ecliptic{113.215630, 6.684170, 0.9765};

  const astro::EquatorialCoord equatorial = astro::equatorialFromEcliptic(ecliptic, obliquity);
  // 同例題の答え: α = 116.328942 度, δ = +28.026183 度
  CHECK_NEAR_ANGLE(equatorial.rightAscensionDegrees, 116.328942, 1e-4);
  CHECK_NEAR(equatorial.declinationDegrees, 28.026183, 1e-4);
  CHECK_NEAR(equatorial.distanceAu, ecliptic.distanceAu, 1e-12);

  const astro::EclipticCoord back = astro::eclipticFromEquatorial(equatorial, obliquity);
  CHECK_NEAR_ANGLE(back.longitudeDegrees, ecliptic.longitudeDegrees, 1e-9);
  CHECK_NEAR(back.latitudeDegrees, ecliptic.latitudeDegrees, 1e-9);
}

void testHorizontalConversion() {
  // Meeus 例題 13.b: 金星を Washington D.C. から観測。
  // α = 23h09m16.641s = 347.3193375 度, δ = -6.719892 度
  // 観測地: 緯度 +38.921389 度、経度は西経 77.065556 度 (東経を正とするので負)
  // グリニッジ視恒星時 128.737873 度 → 地方視恒星時 = 128.737873 - 77.065556 = 51.672318 度
  // 期待値: 方位角 (南basis) = 68.0337 度 → 北basis で 248.0337 度、高度 = 15.1249 度
  astro::EquatorialCoord venus;
  venus.rightAscensionDegrees = 347.3193375;
  venus.declinationDegrees = -6.719892;

  astro::Observer washington;
  washington.latitudeDegrees = 38.921389;
  washington.longitudeEastDegrees = -77.065556;

  const astro::HorizontalCoord horizontal =
      astro::horizontalFromEquatorial(venus, washington, 51.672318);
  CHECK_NEAR_ANGLE(horizontal.azimuthDegrees, 248.0337, 1e-3);
  CHECK_NEAR(horizontal.altitudeDegrees, 15.1249, 1e-3);

  // 天頂の天体: 赤緯 = 緯度、時角 0 なら高度 90 度
  astro::EquatorialCoord zenith;
  zenith.rightAscensionDegrees = 100.0;
  zenith.declinationDegrees = 35.0;
  astro::Observer tokyo;
  tokyo.latitudeDegrees = 35.0;
  tokyo.longitudeEastDegrees = 139.7;
  const astro::HorizontalCoord atZenith = astro::horizontalFromEquatorial(zenith, tokyo, 100.0);
  CHECK_NEAR(atZenith.altitudeDegrees, 90.0, 1e-6);

  // 天の北極は常に真北、高度は観測地の緯度に等しい
  astro::EquatorialCoord pole;
  pole.rightAscensionDegrees = 0.0;
  pole.declinationDegrees = 90.0;
  const astro::HorizontalCoord atPole = astro::horizontalFromEquatorial(pole, tokyo, 123.0);
  CHECK_NEAR(atPole.altitudeDegrees, 35.0, 1e-6);
  CHECK_NEAR_ANGLE(atPole.azimuthDegrees, 0.0, 1e-6);
}

void testParallax() {
  astro::Observer tokyo;
  tokyo.latitudeDegrees = 35.681;
  tokyo.longitudeEastDegrees = 139.767;

  // 月の平均距離 (0.00257 AU) では視差が 0.5 度を超える
  astro::EquatorialCoord moon;
  moon.rightAscensionDegrees = 120.0;
  moon.declinationDegrees = 10.0;
  moon.distanceAu = 0.00257;

  const astro::EquatorialCoord topocentric = astro::applyTopocentricParallax(moon, tokyo, 30.0);
  const double shift = std::hypot(
      astro::angularDifference(moon.rightAscensionDegrees, topocentric.rightAscensionDegrees),
      topocentric.declinationDegrees - moon.declinationDegrees);
  CHECK_TRUE(shift > 0.3);
  CHECK_TRUE(shift < 1.2);

  // 距離未知 (0) なら素通し
  astro::EquatorialCoord unknownDistance;
  unknownDistance.rightAscensionDegrees = 120.0;
  unknownDistance.declinationDegrees = 10.0;
  const astro::EquatorialCoord untouched =
      astro::applyTopocentricParallax(unknownDistance, tokyo, 30.0);
  CHECK_NEAR(untouched.rightAscensionDegrees, 120.0, 1e-12);
  CHECK_NEAR(untouched.declinationDegrees, 10.0, 1e-12);

  // 木星 (約 5 AU) では視差は 1 秒角未満で実質無視できる
  astro::EquatorialCoord jupiter;
  jupiter.rightAscensionDegrees = 120.0;
  jupiter.declinationDegrees = 10.0;
  jupiter.distanceAu = 5.2;
  const astro::EquatorialCoord jupiterTopo = astro::applyTopocentricParallax(jupiter, tokyo, 30.0);
  CHECK_TRUE(std::fabs(jupiterTopo.declinationDegrees - 10.0) < 0.001);
}

void testRefraction() {
  // 地平線上では約 35 分角 (0.586 度) 持ち上がる。
  // Why not 34 分角: 34' は真高度ではなく「見かけの高度」を入力に取る別のモデルの値。
  // Bennett の式は真高度を入力に取るので、地平線での値は 35.17' になる。
  CHECK_NEAR(astro::atmosphericRefractionDegrees(0.0), 0.586, 0.01);
  // 天頂ではほぼゼロ
  CHECK_TRUE(astro::atmosphericRefractionDegrees(90.0) < 0.001);
  // 高度が上がるほど小さくなる (単調減少)
  CHECK_TRUE(astro::atmosphericRefractionDegrees(10.0) > astro::atmosphericRefractionDegrees(45.0));
  // 地平線下ではモデルを適用しない
  CHECK_NEAR(astro::atmosphericRefractionDegrees(-5.0), 0.0, 1e-12);
}

void testExtremeLocations() {
  astro::EquatorialCoord star;
  star.rightAscensionDegrees = 200.0;
  star.declinationDegrees = 45.0;

  // 極地でも NaN を出さない
  for (const double latitude : {89.9, -89.9}) {
    astro::Observer polar;
    polar.latitudeDegrees = latitude;
    polar.longitudeEastDegrees = 0.0;
    const astro::HorizontalCoord horizontal = astro::horizontalFromEquatorial(star, polar, 45.0);
    CHECK_TRUE(!std::isnan(horizontal.azimuthDegrees));
    CHECK_TRUE(!std::isnan(horizontal.altitudeDegrees));
    CHECK_TRUE(horizontal.azimuthDegrees >= 0.0 && horizontal.azimuthDegrees < 360.0);
  }

  // 日付変更線をまたぐ経度でも恒星時が連続している
  const astro::JulianDate now{2461000.5};
  const double justWest = astro::localApparentSiderealTimeDegrees(now, 179.99);
  const double justEast = astro::localApparentSiderealTimeDegrees(now, -179.99);
  CHECK_NEAR_ANGLE(justWest, justEast + 359.98, 1e-6);
}

void testSunAccuracy() {
  // Meeus 例題 25.a/25.b: 1992-10-13 0h TD (JDE 2448908.5)
  const astro::JulianDate example25{2448908.5};
  const astro::EclipticCoord ecliptic = astro::sunEcliptic(example25);
  CHECK_NEAR_ANGLE(ecliptic.longitudeDegrees, 199.90895, 1e-4);
  CHECK_NEAR(ecliptic.distanceAu, 0.99760775, 1e-4);

  const astro::EquatorialCoord equatorial = astro::sunEquatorial(example25);
  astro::EquatorialCoord expected;
  expected.rightAscensionDegrees = 198.380827; // -161.61917 を [0,360) に畳んだ値
  expected.declinationDegrees = -7.78507;
  CHECK_TRUE(angularSeparationDegrees(equatorial, expected) < 0.01);

  // 太陽は常に黄道上 (黄緯ゼロ) で、距離は近日点 0.983 - 遠日点 1.017 の範囲に収まる
  for (int day = 0; day < 365; day += 7) {
    const astro::JulianDate sample{2460000.5 + day};
    const astro::EclipticCoord sun = astro::sunEcliptic(sample);
    CHECK_TRUE(sun.distanceAu > 0.98 && sun.distanceAu < 1.02);
    CHECK_TRUE(sun.longitudeDegrees >= 0.0 && sun.longitudeDegrees < 360.0);
  }
}

void testMoonAccuracy() {
  // Meeus 例題 47.a: 1992-04-12 0h TD (JDE 2448724.5)
  // 書籍の 133.162655 は「真黄経」。ここは章動 Δψ 込みの視黄経を返すので
  // 133.167 付近になるのが正しい。
  const astro::JulianDate example47{2448724.5};
  const astro::EclipticCoord moon = astro::moonEcliptic(example47);
  CHECK_NEAR_ANGLE(moon.longitudeDegrees, 133.162655 + astro::nutation(example47).longitudeDegrees,
                   1e-4);
  CHECK_NEAR(moon.latitudeDegrees, -3.229126, 1e-4);
  CHECK_NEAR(moon.distanceAu * 149597870.7, 368409.7, 1.0);

  // 月の距離は 356400-406700 km の範囲を出ない
  for (int day = 0; day < 60; ++day) {
    const astro::JulianDate sample{2460000.5 + day};
    const double km = astro::moonEcliptic(sample).distanceAu * 149597870.7;
    CHECK_TRUE(km > 356000.0 && km < 407000.0);
    // 黄緯は ±5.3 度を超えない
    CHECK_TRUE(std::fabs(astro::moonEcliptic(sample).latitudeDegrees) < 5.4);
  }
}

void testMoonParallaxMatters() {
  // 視差補正を入れるかどうかで、地上から見た方向が実際に有意に変わることを保証する。
  // これを入れ忘れると月だけ最大 1 度ずれるが、他の天体は正しいので発覚しにくい。
  astro::Observer tokyo;
  tokyo.latitudeDegrees = 35.681;
  tokyo.longitudeEastDegrees = 139.767;

  const astro::JulianDate sample{2460000.5};
  const double siderealTime =
      astro::localApparentSiderealTimeDegrees(sample, tokyo.longitudeEastDegrees);

  const astro::EquatorialCoord geocentric = astro::moonEquatorial(sample);
  const astro::EquatorialCoord topocentric =
      astro::applyTopocentricParallax(geocentric, tokyo, siderealTime);

  const double shift = angularSeparationDegrees(geocentric, topocentric);
  CHECK_TRUE(shift > 0.1); // 補正が確実に効いている
  CHECK_TRUE(shift < 1.1); // かつ物理的に妥当な大きさ (地平視差の上限は約 1 度)
}

void testPlanetAccuracy() {
  // Meeus 例題 33.a: 金星, 1992-12-20 0h TD (JDE 2448976.5) の視位置。
  //
  // 許容値 0.12 度は「JPL 近似要素法の実力」であって目標値ではない。
  // 完全な VSOP87 なら 1 秒角級だが、係数表が数千行になり ESP32 では割に合わない。
  // サーボの分解能が 0.3125 度なので、この誤差は機械側に埋もれる。
  const astro::JulianDate example33{2448976.5};
  const astro::EquatorialCoord venus = astro::planetEquatorial(astro::Planet::Venus, example33);

  astro::EquatorialCoord expected;
  expected.rightAscensionDegrees = 316.172725;
  expected.declinationDegrees = -18.887956;
  CHECK_TRUE(angularSeparationDegrees(venus, expected) < 0.12);

  // 各惑星の地心距離が既知の範囲に収まること (要素表の取り違えを検出する)
  struct DistanceRange {
    astro::Planet planet;
    double minAu;
    double maxAu;
  };
  const DistanceRange ranges[] = {
      {astro::Planet::Mercury, 0.5, 1.5}, {astro::Planet::Venus, 0.25, 1.75},
      {astro::Planet::Mars, 0.35, 2.7},   {astro::Planet::Jupiter, 3.9, 6.5},
      {astro::Planet::Saturn, 7.9, 11.1},
  };
  for (const DistanceRange& range : ranges) {
    for (int day = 0; day < 800; day += 40) {
      const astro::JulianDate sample{2460000.5 + day};
      const astro::EquatorialCoord position = astro::planetEquatorial(range.planet, sample);
      CHECK_TRUE(position.distanceAu > range.minAu);
      CHECK_TRUE(position.distanceAu < range.maxAu);
    }
  }
}

void testKeplerSolver() {
  // 円軌道では離心近点角 = 平均近点角
  CHECK_NEAR(astro::solveKeplerEccentricAnomalyRadians(1.0, 0.0), 1.0, 1e-12);
  // 解いた E が元の方程式を満たす (水星の離心率でも収束する)
  for (const double eccentricity : {0.0, 0.05, 0.21, 0.5}) {
    for (const double meanAnomaly : {-3.0, -1.0, 0.0, 0.5, 2.0, 3.1}) {
      const double eccentricAnomaly =
          astro::solveKeplerEccentricAnomalyRadians(meanAnomaly, eccentricity);
      const double residual =
          eccentricAnomaly - eccentricity * std::sin(eccentricAnomaly) - meanAnomaly;
      CHECK_NEAR(residual, 0.0, 1e-9);
    }
  }
}

void testEphemerisFacade() {
  astro::Observer tokyo;
  tokyo.latitudeDegrees = 35.681;
  tokyo.longitudeEastDegrees = 139.767;

  // 真北は時計が無くても必ず解ける (時刻同期に失敗しても方位だけは示せる)
  const astro::TargetPosition north =
      astro::computeTargetPosition(astro::Target::North, 0, tokyo, false);
  CHECK_TRUE(north.valid);
  CHECK_NEAR(north.horizontal.azimuthDegrees, 0.0, 1e-12);
  CHECK_NEAR(north.horizontal.altitudeDegrees, 0.0, 1e-12);

  // 天体は時刻が無いと解けない
  const astro::TargetPosition sunWithoutTime =
      astro::computeTargetPosition(astro::Target::Sun, 0, tokyo, false);
  CHECK_TRUE(!sunWithoutTime.valid);

  // 2026-08-11 12:00 JST = 03:00 UTC。東京の夏の南中前後なので、
  // 太陽は高くて南寄りにいるはず。
  const std::int64_t noonJst = 1786064400;
  const astro::TargetPosition sun =
      astro::computeTargetPosition(astro::Target::Sun, noonJst, tokyo, true);
  CHECK_TRUE(sun.valid);
  CHECK_TRUE(sun.aboveHorizon);
  CHECK_TRUE(sun.horizontal.altitudeDegrees > 50.0);
  CHECK_TRUE(sun.horizontal.azimuthDegrees > 90.0 && sun.horizontal.azimuthDegrees < 270.0);

  // 真夜中の太陽は地平線下にいる (指し示しはするが下を向く)
  const astro::TargetPosition midnightSun =
      astro::computeTargetPosition(astro::Target::Sun, noonJst + 43200, tokyo, true);
  CHECK_TRUE(midnightSun.valid);
  CHECK_TRUE(!midnightSun.aboveHorizon);
  CHECK_TRUE(midnightSun.horizontal.altitudeDegrees < 0.0);

  // 全ターゲットが解けて、方位が [0,360) に収まる
  for (std::uint8_t index = 0; index < static_cast<std::uint8_t>(astro::Target::kCount); ++index) {
    const auto target = static_cast<astro::Target>(index);
    const astro::TargetPosition position =
        astro::computeTargetPosition(target, noonJst, tokyo, true);
    CHECK_TRUE(position.valid);
    CHECK_TRUE(position.horizontal.azimuthDegrees >= 0.0);
    CHECK_TRUE(position.horizontal.azimuthDegrees < 360.0);
    CHECK_TRUE(std::fabs(position.horizontal.altitudeDegrees) <= 90.5);
    CHECK_TRUE(!std::isnan(position.horizontal.azimuthDegrees));
  }
}

} // namespace

int main() {
  testAngleNormalization();
  testJulianDate();
  testSiderealTime();
  testObliquity();
  testEclipticEquatorialRoundTrip();
  testHorizontalConversion();
  testParallax();
  testRefraction();
  testExtremeLocations();
  testSunAccuracy();
  testMoonAccuracy();
  testMoonParallaxMatters();
  testPlanetAccuracy();
  testKeplerSolver();
  testEphemerisFacade();
  return testing::summarize("astro");
}
