#include "astro/ephemeris.hpp"

#include "astro/ephemeris_db.hpp"
#include "astro/moon.hpp"
#include "astro/planets.hpp"
#include "astro/sun.hpp"
#include "astro/time.hpp"

namespace astro {
namespace {

bool targetAsEphemerisBody(Target target, EphemerisBody& bodyOut) {
  switch (target) {
  case Target::Sun:
    bodyOut = EphemerisBody::Sun;
    return true;
  case Target::Moon:
    bodyOut = EphemerisBody::Moon;
    return true;
  case Target::Mercury:
    bodyOut = EphemerisBody::Mercury;
    return true;
  case Target::Venus:
    bodyOut = EphemerisBody::Venus;
    return true;
  case Target::Mars:
    bodyOut = EphemerisBody::Mars;
    return true;
  case Target::Jupiter:
    bodyOut = EphemerisBody::Jupiter;
    return true;
  case Target::Saturn:
    bodyOut = EphemerisBody::Saturn;
    return true;
  case Target::North:
  case Target::kCount:
    return false;
  }
  return false;
}

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

const char* ephemerisSourceName(EphemerisSource source) {
  switch (source) {
  case EphemerisSource::None:
    return "none";
  case EphemerisSource::FixedDirection:
    return "fixed";
  case EphemerisSource::HighPrecisionDatabase:
    return "jpl-db";
  case EphemerisSource::ApproximateModel:
    return "approx";
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
    result.source = EphemerisSource::FixedDirection;
    return result;
  }

  if (!timeValid) {
    return result;
  }

  const JulianDate julianDate = julianDateFromUnixSeconds(unixSeconds);
  const double siderealTime =
      localApparentSiderealTimeDegrees(julianDate, observer.longitudeEastDegrees);

  EquatorialCoord equatorial;
  EphemerisBody databaseBody = EphemerisBody::Sun;
  const bool hasDatabaseBody = targetAsEphemerisBody(target, databaseBody);
  const bool hasDatabasePosition =
      hasDatabaseBody && lookupHighPrecisionEquatorial(databaseBody, unixSeconds, equatorial);
  if (hasDatabasePosition) {
    result.source = EphemerisSource::HighPrecisionDatabase;
  } else {
    Planet planet = Planet::Mercury;
    const bool isSun = target == Target::Sun;
    const bool isMoon = target == Target::Moon;
    const bool isPlanet = targetIsPlanet(target, planet);
    if (isSun) {
      equatorial = sunEquatorial(julianDate);
    } else if (isMoon) {
      equatorial = moonEquatorial(julianDate);
    } else if (isPlanet) {
      equatorial = planetEquatorial(planet, julianDate);
    } else {
      return result;
    }
    result.source = EphemerisSource::ApproximateModel;
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
