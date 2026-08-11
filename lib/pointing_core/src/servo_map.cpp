#include "pointing/servo_map.hpp"

#include <algorithm>
#include <cmath>

namespace pointing {
namespace {

double normalizeSignedDegrees(double degrees) {
  double wrapped = std::fmod(degrees, 360.0);
  if (wrapped <= -180.0) {
    wrapped += 360.0;
  }
  if (wrapped > 180.0) {
    wrapped -= 360.0;
  }
  return wrapped;
}

} // namespace

int pitchDeciFromAltitude(double altitudeDegrees) {
  const double raw =
      static_cast<double>(kPitchLevelDeci) + altitudeDegrees * kDeciDegreesPerAltitudeDegree;
  const long rounded = std::lround(raw);
  return static_cast<int>(std::clamp<long>(rounded, kPitchMinDeci, kPitchMaxDeci));
}

double altitudeFromPitchDeci(int pitchDeci) {
  return static_cast<double>(pitchDeci - kPitchLevelDeci) / kDeciDegreesPerAltitudeDegree;
}

int yawDeciFromRelativeBearing(double relativeBearingDegrees) {
  const double signed180 = normalizeSignedDegrees(relativeBearingDegrees);
  const long rounded = std::lround(signed180 * 10.0);
  return static_cast<int>(std::clamp<long>(rounded, kYawMinDeci, kYawMaxDeci));
}

bool isYawReachable(int yawDeciDegrees) {
  return yawDeciDegrees >= kYawMinDeci && yawDeciDegrees <= kYawMaxDeci;
}

bool isPitchReachable(int pitchDeciDegrees) {
  return pitchDeciDegrees >= kPitchMinDeci && pitchDeciDegrees <= kPitchMaxDeci;
}

bool isServoAtTarget(int actualYawDeciDegrees, int actualPitchDeciDegrees, int targetYawDeciDegrees,
                     int targetPitchDeciDegrees, int toleranceDeciDegrees) {
  const bool yawReached =
      std::abs(actualYawDeciDegrees - targetYawDeciDegrees) <= toleranceDeciDegrees;
  const bool pitchReached =
      std::abs(actualPitchDeciDegrees - targetPitchDeciDegrees) <= toleranceDeciDegrees;
  return yawReached && pitchReached;
}

} // namespace pointing
