#pragma once

#include "astro/coords.hpp"

#include <cstdint>

namespace astro {

// 高精度暦DBで扱う天体。地球は観測中心としてJPL側で差し引かれているため含めない。
enum class EphemerisBody : std::uint8_t {
  Sun = 0,
  Moon,
  Mercury,
  Venus,
  Mars,
  Jupiter,
  Saturn,
  kCount,
};

struct EphemerisDatabaseInfo {
  std::int64_t startUnixSeconds = 0;
  std::int64_t stopUnixSeconds = 0;
  const char* source = "";
  std::uint32_t dataCrc32 = 0;
};

// ビルドへ埋め込まれたJPL暦DBの由来と有効期間。
[[nodiscard]] const EphemerisDatabaseInfo& ephemerisDatabaseInfo();

// 地心・観測日分点の見かけ赤道座標をDBから4点補間する。
// DB期間外またはデータ不正時は false を返し、呼び出し側が近似計算へ戻れるようにする。
[[nodiscard]] bool lookupHighPrecisionEquatorial(EphemerisBody body, std::int64_t unixSeconds,
                                                 EquatorialCoord& result);

} // namespace astro
