#include "pointing/solver.hpp"

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

SolveResult solve(const SolveInput& input) {
  SolveResult result;
  result.belowHorizon = input.targetAltitudeDegrees < 0.0;

  // 機体正面から見て、目標がどれだけ横にいるか。
  const double relativeBearing =
      normalizeSignedDegrees(input.targetAzimuthDegrees - input.bodyHeadingDegrees);
  const int yawDeci = yawDeciFromRelativeBearing(relativeBearing);

  // clamp された分は「首では届かなかった量」として残す。
  const double requestedYawDeci = relativeBearing * 10.0;
  result.command.yawDeciDegrees = yawDeci;
  result.command.clampedYaw = std::fabs(requestedYawDeci - static_cast<double>(yawDeci)) > 0.5;
  result.unreachableYawDegrees =
      result.command.clampedYaw ? (requestedYawDeci - static_cast<double>(yawDeci)) / 10.0 : 0.0;

  double altitudeToPoint = input.targetAltitudeDegrees;
  if (result.belowHorizon && input.belowHorizonPolicy == BelowHorizonPolicy::ClampToLevel) {
    altitudeToPoint = 0.0;
  }

  const int pitchDeci = pitchDeciFromAltitude(altitudeToPoint);
  const double requestedPitchDeci =
      static_cast<double>(kPitchLevelDeci) + altitudeToPoint * kDeciDegreesPerAltitudeDegree;
  result.command.pitchDeciDegrees = pitchDeci;
  result.command.clampedPitch =
      std::fabs(requestedPitchDeci - static_cast<double>(pitchDeci)) > 0.5;
  result.unreachablePitchDegrees =
      result.command.clampedPitch
          ? (requestedPitchDeci - static_cast<double>(pitchDeci)) / kDeciDegreesPerAltitudeDegree
          : 0.0;

  // deadband: 現在位置からの変化が小さいうちは動かさない。
  // サーボを鳴かせ続けないためと、磁気測定の機会を確保するため。
  const double yawChangeDegrees =
      std::fabs(static_cast<double>(yawDeci - input.currentYawDeciDegrees)) / 10.0;
  const double pitchChangeDegrees =
      std::fabs(static_cast<double>(pitchDeci - input.currentPitchDeciDegrees)) /
      kDeciDegreesPerAltitudeDegree;
  result.shouldMove =
      yawChangeDegrees >= input.deadbandDegrees || pitchChangeDegrees >= input.deadbandDegrees;

  return result;
}

bool needsBodyRotation(const SolveResult& result) { return result.command.clampedYaw; }

} // namespace pointing
