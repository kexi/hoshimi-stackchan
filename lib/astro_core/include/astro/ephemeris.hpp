#pragma once

#include "astro/coords.hpp"

#include <cstdint>

namespace astro {

// スワイプで順送りする対象。North を先頭に置き、以降を天体にする。
enum class Target : std::uint8_t {
  North = 0,
  Sun,
  Moon,
  Mercury,
  Venus,
  Mars,
  Jupiter,
  Saturn,
  kCount,
};

[[nodiscard]] const char* targetName(Target target);

enum class EphemerisSource : std::uint8_t {
  None = 0,
  FixedDirection,
  HighPrecisionDatabase,
  ApproximateModel,
};

[[nodiscard]] const char* ephemerisSourceName(EphemerisSource source);

struct TargetPosition {
  HorizontalCoord horizontal;
  bool aboveHorizon = false;
  // 時刻が未同期のとき、天体は解けない (North だけは常に解ける)。
  bool valid = false;
  EphemerisSource source = EphemerisSource::None;
};

// ファームウェアから見た唯一の入口。
// 返す方位角は真北基準・大気差補正込み。Target::North は方位 0・高度 0 を返す。
[[nodiscard]] TargetPosition computeTargetPosition(Target target, std::int64_t unixSeconds,
                                                   Observer observer, bool timeValid);

} // namespace astro
