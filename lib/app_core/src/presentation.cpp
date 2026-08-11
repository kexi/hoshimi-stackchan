#include "app/presentation.hpp"

#include <cstdio>

namespace app {
namespace {

// 指せているときのセリフ。Avatar の吹き出しが画面内に収まるよう、全角換算で
// 6文字以内にする。
const char* pointingSpeech(astro::Target target) {
  switch (target) {
  case astro::Target::North:
    return "きたはこっち";
  case astro::Target::Sun:
    return "おひさま！";
  case astro::Target::Moon:
    return "月が綺麗だね";
  case astro::Target::Mercury:
    return "水星いるよ";
  case astro::Target::Venus:
    return "金星ピカピカ";
  case astro::Target::Mars:
    return "火星は赤いよ";
  case astro::Target::Jupiter:
    return "木星おおきい";
  case astro::Target::Saturn:
    return "土星のわっか";
  case astro::Target::kCount:
    break;
  }
  return "あっちだよ";
}

} // namespace

const char* targetNameJapanese(astro::Target target) {
  switch (target) {
  case astro::Target::North:
    return "北";
  case astro::Target::Sun:
    return "太陽";
  case astro::Target::Moon:
    return "月";
  case astro::Target::Mercury:
    return "水星";
  case astro::Target::Venus:
    return "金星";
  case astro::Target::Mars:
    return "火星";
  case astro::Target::Jupiter:
    return "木星";
  case astro::Target::Saturn:
    return "土星";
  case astro::Target::kCount:
    break;
  }
  return "?";
}

FacePresentation facePresentationFor(const State& state, float calibrationCoverage) {
  FacePresentation presentation;

  const bool isError = state.phase == Phase::Error;
  if (isError) {
    presentation.mood = FaceMood::Doubt;
    std::snprintf(presentation.speech.data(), presentation.speech.size(), "メモリ不足");
    return presentation;
  }

  const bool isCalibrating = state.phase == Phase::Calibrating;
  if (isCalibrating) {
    presentation.mood = FaceMood::Doubt;
    std::snprintf(presentation.speech.data(), presentation.speech.size(), "回して %d%%",
                  static_cast<int>(calibrationCoverage * 100.0F));
    return presentation;
  }

  const bool isClockUnavailable = !state.lastPosition.valid;
  if (isClockUnavailable) {
    presentation.mood = FaceMood::Sleepy;
    std::snprintf(presentation.speech.data(), presentation.speech.size(), "時計がないの");
    return presentation;
  }

  const char* const targetName = targetNameJapanese(state.target);
  const bool isBelowHorizon = !state.lastPosition.aboveHorizon;
  if (isBelowHorizon) {
    // Why not pitchの可動域を先に見る: 地平線下の天体はpitch下限にも掛かるため、
    // 先に可動域を見ると「上すぎ」と誤案内する。対象を主語にして原因を伝える。
    presentation.mood = FaceMood::Sleepy;
    std::snprintf(presentation.speech.data(), presentation.speech.size(), "%sは地平下", targetName);
    return presentation;
  }

  const bool isBehindYawLimit = state.lastSolve.command.clampedYaw;
  if (isBehindYawLimit) {
    presentation.mood = FaceMood::Doubt;
    std::snprintf(presentation.speech.data(), presentation.speech.size(), "%sはうしろ", targetName);
    return presentation;
  }

  const bool isAbovePitchLimit = state.lastSolve.command.clampedPitch;
  if (isAbovePitchLimit) {
    presentation.mood = FaceMood::Doubt;
    std::snprintf(presentation.speech.data(), presentation.speech.size(), "%sは上すぎ", targetName);
    return presentation;
  }

  presentation.mood = FaceMood::Happy;
  std::snprintf(presentation.speech.data(), presentation.speech.size(), "%s",
                pointingSpeech(state.target));
  return presentation;
}

} // namespace app
