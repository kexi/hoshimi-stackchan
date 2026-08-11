#pragma once

namespace astro {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

[[nodiscard]] double degToRad(double degrees);
[[nodiscard]] double radToDeg(double radians);

// [0, 360) に畳む。
[[nodiscard]] double normalizeDegrees(double degrees);

// (-180, 180] に畳む。
[[nodiscard]] double normalizeSignedDegrees(double degrees);

// from から to への最短回転量 (-180, 180]。正が反時計回り側ではなく
// 「度数の増える向き」であることに注意 (方位角なら東回り)。
[[nodiscard]] double angularDifference(double fromDegrees, double toDegrees);

} // namespace astro
