#include "compass/declination.hpp"

#include <cmath>

namespace compass {
namespace {

float normalizeDegrees(float degrees) {
  float wrapped = std::fmod(degrees, 360.0F);
  if (wrapped < 0.0F) {
    wrapped += 360.0F;
  }
  if (wrapped >= 360.0F) {
    wrapped = 0.0F;
  }
  return wrapped;
}

} // namespace

float trueHeadingFromMagnetic(float magneticHeadingDegrees, float declinationEastDegrees) {
  return normalizeDegrees(magneticHeadingDegrees + declinationEastDegrees);
}

float magneticHeadingFromTrue(float trueHeadingDegrees, float declinationEastDegrees) {
  return normalizeDegrees(trueHeadingDegrees - declinationEastDegrees);
}

} // namespace compass
