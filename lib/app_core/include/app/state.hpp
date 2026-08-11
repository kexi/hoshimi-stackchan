#pragma once

#include "astro/ephemeris.hpp"
#include "compass/stability.hpp"
#include "pointing/solver.hpp"

#include <cstdint>

namespace app {

// 状態機械。磁気測定とサーボ動作は排他でなければならないので、
// 並行タスクには分けず 1 本のシーケンスに閉じる。
enum class Phase : std::uint8_t {
  InitHardware,
  ConnectWifi,
  SyncTime,
  Calibrating,
  Idle,
  // 首を正面へ戻す。ここを挟まないと、首の角度によって方位が最大 119 度ずれる
  // (段階 8 実測)。磁気測定の前提を揃えるための姿勢。
  ReturningToMeasurePose,
  Measuring,
  Pointing,
  Tracking,
  Error,
};

[[nodiscard]] const char* phaseName(Phase phase);

// スワイプの向き。
enum class Input : std::uint8_t {
  None,
  SwipeForward,
  SwipeBackward,
  Click,
};

struct Config {
  // 方位を測り直す間隔。天体は動くが、方位は機体が動かない限り変わらない。
  std::uint32_t remeasureIntervalMillis = 60000;
  // 自動巡回時に次のターゲットへ移る間隔。
  std::uint32_t autoCycleIntervalMillis = 8000;
  // サーボ指令を出してから収束したとみなすまでの時間。
  std::uint32_t servoSettleMillis = 1200;
  // 首を正面に戻してから磁気を測り始めるまでの待ち。実測の収束 300ms に
  // 首の移動時間を足した値。
  std::uint32_t measurePoseSettleMillis = 1000;
  // 測定に使うサンプル数と、諦めるまでの時間。
  std::uint32_t measureTimeoutMillis = 5000;
  // 機体が動かされたと判断するジャイロのしきい値。
  float bodyMovedGyroDegPerSec = 30.0F;
};

// 実機・シミュレータの双方から同じ形で渡す入力。
struct Tick {
  std::uint32_t nowMillis = 0;
  std::int64_t unixSeconds = 0;
  bool timeValid = false;
  bool calibrationValid = false;
  Input input = Input::None;

  // 直近に採用された方位 (真方位)。headingValid が false なら未取得。
  float bodyTrueHeadingDegrees = 0.0F;
  bool headingValid = false;

  // 測定ゲートの判定結果。Measuring 中の遷移に使う。
  bool measurementAccepted = false;
  compass::MeasurementGate::Reject lastReject = compass::MeasurementGate::Reject::None;

  float gyroMagnitudeDegPerSec = 0.0F;
  bool servoSettled = true;
};

struct State {
  Phase phase = Phase::InitHardware;
  astro::Target target = astro::Target::North;
  bool autoCycleEnabled = false;

  std::uint32_t phaseEnteredMillis = 0;
  std::uint32_t lastMeasureMillis = 0;
  std::uint32_t lastTargetSwitchMillis = 0;

  astro::TargetPosition lastPosition;
  pointing::SolveResult lastSolve;
  bool hasSolve = false;

  compass::MeasurementGate::Reject lastReject = compass::MeasurementGate::Reject::None;
};

// 今この瞬間、首をどこへ向けるべきか。ファームはこの指令をそのまま
// Motion.move() に渡す。測定中は必ず正面に戻る。
struct ServoIntent {
  int yawDeciDegrees = 0;
  int pitchDeciDegrees = pointing::kPitchLevelDeci;
  bool shouldMove = false;
};

[[nodiscard]] ServoIntent servoIntentFor(const State& state);

// 状態機械を 1 ステップ進める。副作用は持たず、State を書き換えるだけ。
// サーボへの指令は State.lastSolve に入るので、呼び出し側が実際に動かす。
void step(State& state, const Tick& tick, const Config& config, const astro::Observer& observer);

[[nodiscard]] astro::Target nextTarget(astro::Target current);
[[nodiscard]] astro::Target previousTarget(astro::Target current);

} // namespace app
