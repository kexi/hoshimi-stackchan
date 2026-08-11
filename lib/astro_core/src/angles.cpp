#include "astro/angles.hpp"

#include <cmath>

namespace astro {

double degToRad(double degrees) { return degrees * kDeg2Rad; }

double radToDeg(double radians) { return radians * kRad2Deg; }

double normalizeDegrees(double degrees) {
  double wrapped = std::fmod(degrees, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  // fmod は -1e-18 のような極小の負値に対して 360.0 を返しうる。
  // [0,360) の約束を守るため 0 に落とす。
  if (wrapped >= 360.0) {
    wrapped = 0.0;
  }
  return wrapped;
}

double normalizeSignedDegrees(double degrees) {
  double wrapped = normalizeDegrees(degrees);
  if (wrapped > 180.0) {
    wrapped -= 360.0;
  }
  return wrapped;
}

double angularDifference(double fromDegrees, double toDegrees) {
  return normalizeSignedDegrees(toDegrees - fromDegrees);
}

} // namespace astro
