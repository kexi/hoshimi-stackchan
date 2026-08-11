#include "astro/ephemeris_db.hpp"

#include "astro/angles.hpp"

#include <cmath>
#include <cstddef>

namespace astro {
namespace {

struct StoredSample {
  float rightAscensionDegrees;
  float declinationDegrees;
  float distanceAu;
};

struct StoredSeries {
  std::int64_t startUnixSeconds;
  std::uint32_t stepSeconds;
  std::size_t sampleCount;
  const StoredSample* samples;
};

// このファイルは scripts/generate_ephemeris_db.py がJPL Horizonsから生成する。
// 通常ビルドは生成済みデータだけを使うので、実機にネット接続は要らない。
#include "ephemeris_db_data.inc"

constexpr std::size_t kGeneratedSeriesCount =
    sizeof(kGeneratedSeries) / sizeof(kGeneratedSeries[0]);
static_assert(kGeneratedSeriesCount == static_cast<std::size_t>(EphemerisBody::kCount),
              "生成DBの天体数がEphemerisBodyと一致しません");

double lagrange4(double y0, double y1, double y2, double y3, double x) {
  const double weight0 = -((x - 1.0) * (x - 2.0) * (x - 3.0)) / 6.0;
  const double weight1 = (x * (x - 2.0) * (x - 3.0)) / 2.0;
  const double weight2 = -(x * (x - 1.0) * (x - 3.0)) / 2.0;
  const double weight3 = (x * (x - 1.0) * (x - 2.0)) / 6.0;
  return weight0 * y0 + weight1 * y1 + weight2 * y2 + weight3 * y3;
}

const StoredSeries* seriesFor(EphemerisBody body) {
  const std::size_t index = static_cast<std::size_t>(body);
  const bool isKnownBody = index < static_cast<std::size_t>(EphemerisBody::kCount);
  if (!isKnownBody) {
    return nullptr;
  }
  return &kGeneratedSeries[index];
}

std::size_t interpolationStartIndex(double samplePosition, std::size_t sampleCount) {
  const std::size_t lowerIndex = static_cast<std::size_t>(std::floor(samplePosition));
  std::size_t startIndex = lowerIndex > 0 ? lowerIndex - 1 : 0;
  const bool extendsPastEnd = startIndex + 3 >= sampleCount;
  if (extendsPastEnd) {
    startIndex = sampleCount - 4;
  }
  return startIndex;
}

double unwrappedRightAscension(const StoredSample& sample, double referenceDegrees) {
  return referenceDegrees + angularDifference(referenceDegrees, sample.rightAscensionDegrees);
}

} // namespace

const EphemerisDatabaseInfo& ephemerisDatabaseInfo() { return kGeneratedDatabaseInfo; }

bool lookupHighPrecisionEquatorial(EphemerisBody body, std::int64_t unixSeconds,
                                   EquatorialCoord& result) {
  const StoredSeries* series = seriesFor(body);
  const bool hasEnoughSamples = series != nullptr && series->sampleCount >= 4;
  if (!hasEnoughSamples) {
    return false;
  }

  const std::int64_t lastUnixSeconds =
      series->startUnixSeconds + static_cast<std::int64_t>(series->stepSeconds) *
                                     static_cast<std::int64_t>(series->sampleCount - 1);
  const bool isInsideDatabase =
      unixSeconds >= series->startUnixSeconds && unixSeconds <= lastUnixSeconds;
  if (!isInsideDatabase) {
    return false;
  }

  const double samplePosition = static_cast<double>(unixSeconds - series->startUnixSeconds) /
                                static_cast<double>(series->stepSeconds);
  const std::size_t startIndex = interpolationStartIndex(samplePosition, series->sampleCount);
  const double x = samplePosition - static_cast<double>(startIndex);

  const StoredSample& sample0 = series->samples[startIndex];
  const StoredSample& sample1 = series->samples[startIndex + 1];
  const StoredSample& sample2 = series->samples[startIndex + 2];
  const StoredSample& sample3 = series->samples[startIndex + 3];

  // RAだけは0/360度をまたぐため、中央寄りの点を基準に連続角へほどいてから補間する。
  const double referenceRa = sample1.rightAscensionDegrees;
  const double ra0 = unwrappedRightAscension(sample0, referenceRa);
  const double ra1 = unwrappedRightAscension(sample1, referenceRa);
  const double ra2 = unwrappedRightAscension(sample2, referenceRa);
  const double ra3 = unwrappedRightAscension(sample3, referenceRa);

  EquatorialCoord interpolated;
  interpolated.rightAscensionDegrees = normalizeDegrees(lagrange4(ra0, ra1, ra2, ra3, x));
  interpolated.declinationDegrees =
      lagrange4(sample0.declinationDegrees, sample1.declinationDegrees, sample2.declinationDegrees,
                sample3.declinationDegrees, x);
  interpolated.distanceAu =
      lagrange4(sample0.distanceAu, sample1.distanceAu, sample2.distanceAu, sample3.distanceAu, x);

  const bool isFinite = std::isfinite(interpolated.rightAscensionDegrees) &&
                        std::isfinite(interpolated.declinationDegrees) &&
                        std::isfinite(interpolated.distanceAu);
  const bool isPhysicallyValid = interpolated.declinationDegrees >= -90.0 &&
                                 interpolated.declinationDegrees <= 90.0 &&
                                 interpolated.distanceAu > 0.0;
  const bool isValid = isFinite && isPhysicallyValid;
  if (!isValid) {
    return false;
  }

  result = interpolated;
  return true;
}

} // namespace astro
