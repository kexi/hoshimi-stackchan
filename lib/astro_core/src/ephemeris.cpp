#include "astro/ephemeris.hpp"

#include "astro/moon.hpp"
#include "astro/planets.hpp"
#include "astro/sun.hpp"
#include "astro/time.hpp"

namespace astro {
namespace {

bool targetIsPlanet(Target target, Planet& planetOut) {
  switch (target) {
  case Target::Mercury:
    planetOut = Planet::Mercury;
    return true;
  case Target::Venus:
    planetOut = Planet::Venus;
    return true;
  case Target::Mars:
    planetOut = Planet::Mars;
    return true;
  case Target::Jupiter:
    planetOut = Planet::Jupiter;
    return true;
  case Target::Saturn:
    planetOut = Planet::Saturn;
    return true;
  default:
    return false;
  }
}

} // namespace

const char* targetName(Target target) {
  switch (target) {
  case Target::North:
    return "North";
  case Target::Sun:
    return "Sun";
  case Target::Moon:
    return "Moon";
  case Target::Mercury:
    return "Mercury";
  case Target::Venus:
    return "Venus";
  case Target::Mars:
    return "Mars";
  case Target::Jupiter:
    return "Jupiter";
  case Target::Saturn:
    return "Saturn";
  case Target::kCount:
    break;
  }
  return "?";
}

TargetPosition computeTargetPosition(Target target, std::int64_t unixSeconds, Observer observer,
                                     bool timeValid) {
  TargetPosition result;

  // 真北は時計も暦も要らない。時刻同期に失敗しても必ず指せる。
  if (target == Target::North) {
    result.horizontal.azimuthDegrees = 0.0;
    result.horizontal.altitudeDegrees = 0.0;
    result.aboveHorizon = true;
    result.valid = true;
    return result;
  }

  if (!timeValid) {
    return result;
  }

  const JulianDate julianDate = julianDateFromUnixSeconds(unixSeconds);
  const double siderealTime =
      localApparentSiderealTimeDegrees(julianDate, observer.longitudeEastDegrees);

  EquatorialCoord equatorial;
  Planet planet = Planet::Mercury;
  if (target == Target::Sun) {
    equatorial = sunEquatorial(julianDate);
  } else if (target == Target::Moon) {
    equatorial = moonEquatorial(julianDate);
  } else if (targetIsPlanet(target, planet)) {
    equatorial = planetEquatorial(planet, julianDate);
  } else {
    return result;
  }

  // 月は視差が最大 1 度に達するので、地心のままでは方位精度 0.1 度を満たせない。
  // 太陽・惑星でも視差は入れて損はない (遠いほど自動的にゼロに近づく)。
  const EquatorialCoord topocentric = applyTopocentricParallax(equatorial, observer, siderealTime);

  HorizontalCoord horizontal = horizontalFromEquatorial(topocentric, observer, siderealTime);

  // 大気差は「実際にどちらを向けば見えるか」に効くので、指し示す前に足す。
  horizontal.altitudeDegrees += atmosphericRefractionDegrees(horizontal.altitudeDegrees);

  result.horizontal = horizontal;
  result.aboveHorizon = horizontal.altitudeDegrees > 0.0;
  result.valid = true;
  return result;
}

} // namespace astro
