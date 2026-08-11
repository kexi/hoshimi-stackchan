#pragma once

#include "app/state.hpp"

#include <array>
#include <cstdint>

namespace app {

enum class FaceMood : std::uint8_t {
  Doubt,
  Sleepy,
  Happy,
};

struct FacePresentation {
  FaceMood mood = FaceMood::Doubt;
  std::array<char, 64> speech{};
};

// 状態から表情と吹き出しを決める。描画ライブラリには依存させず、ホストの
// テストでも実機と同じ文言を検証できるようにする。
[[nodiscard]] FacePresentation facePresentationFor(const State& state, float calibrationCoverage);

[[nodiscard]] const char* targetNameJapanese(astro::Target target);

} // namespace app
