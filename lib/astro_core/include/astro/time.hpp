#pragma once

#include <cstdint>

namespace astro {

// ユリウス日。UT1 と UTC の差 (DUT1、最大 0.9 秒) は無視する。
// 恒星時に落として 0.004 度で、方位精度 0.1 度の要求内に収まる。
struct JulianDate {
  double jd = 0.0;

  // J2000.0 からのユリウス世紀 T
  [[nodiscard]] double centuriesSinceJ2000() const;
  // J2000.0 からのユリウス日数 d
  [[nodiscard]] double daysSinceJ2000() const;
};

[[nodiscard]] JulianDate julianDateFromUnixSeconds(std::int64_t unixSeconds);
[[nodiscard]] std::int64_t unixSecondsFromJulianDate(JulianDate julianDate);

// グリニッジ平均恒星時 [度]。Meeus (12.4)。
[[nodiscard]] double greenwichMeanSiderealTimeDegrees(JulianDate julianDate);

// 地方平均恒星時 [度]。東経を正とする。
[[nodiscard]] double localMeanSiderealTimeDegrees(JulianDate julianDate,
                                                  double longitudeEastDegrees);

// 章動による赤経補正 (equation of the equinoxes) 込みの地方視恒星時 [度]。
// 補正は最大 1.1 秒角 = 0.0003 度で要求に対しては無視できるが、数行なので入れる。
[[nodiscard]] double localApparentSiderealTimeDegrees(JulianDate julianDate,
                                                      double longitudeEastDegrees);

// 章動。Meeus ch.22 の主要項のみ。
struct Nutation {
  double longitudeDegrees = 0.0; // Δψ
  double obliquityDegrees = 0.0; // Δε
};
[[nodiscard]] Nutation nutation(JulianDate julianDate);

// 平均黄道傾斜角 ε0 [度]。Meeus (22.2)。
[[nodiscard]] double meanObliquityDegrees(JulianDate julianDate);

// 章動込みの真黄道傾斜角 ε [度]。
[[nodiscard]] double trueObliquityDegrees(JulianDate julianDate);

} // namespace astro
