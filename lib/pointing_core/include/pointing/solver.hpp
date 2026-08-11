#pragma once

#include "pointing/servo_map.hpp"

#include <cstdint>

namespace pointing {

// 地平線下の天体をどう指すか。
enum class BelowHorizonPolicy : std::uint8_t {
  // 方位はそのまま指し、pitch は高度に応じて下を向く。
  // 真夜中の太陽の方位は「地球の裏側のこっち」で、教育的に正しい。
  PointAnyway,
  // 方位はそのまま、pitch は水平で止める。
  ClampToLevel,
};

struct SolveInput {
  double targetAzimuthDegrees = 0.0; // 真方位
  double targetAltitudeDegrees = 0.0;
  double bodyHeadingDegrees = 0.0; // 機体正面が向いている真方位
  int currentYawDeciDegrees = 0;
  int currentPitchDeciDegrees = kPitchLevelDeci;
  BelowHorizonPolicy belowHorizonPolicy = BelowHorizonPolicy::PointAnyway;
  // これ未満の変化では動かさない。
  //
  // サーボの分解能 0.3125 度より粗くするだけでなく、方位測定のばらつきより
  // 大きくする必要がある。バイアス補正後の方位は実機で 12 度ほど揺れており、
  // 0.5 度だと毎周期 shouldMove が立って首が小刻みに揺れ続けた。指す精度は
  // ±6 度に収まっているので、その範囲では動かさない。
  double deadbandDegrees = 6.0;
};

struct SolveResult {
  ServoCommand command;
  bool shouldMove = false;
  bool belowHorizon = false;
  // 可動域外で「本当は指せていない」量 [度]。UI が伝えるために使う。
  double unreachableYawDegrees = 0.0;
  double unreachablePitchDegrees = 0.0;
};

[[nodiscard]] SolveResult solve(const SolveInput& input);

// 首だけでは届かず、体ごと回してもらう必要があるか。
[[nodiscard]] bool needsBodyRotation(const SolveResult& result);

} // namespace pointing
