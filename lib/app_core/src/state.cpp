#include "app/state.hpp"

namespace app {
namespace {

void enterPhase(State& state, Phase phase, std::uint32_t nowMillis) {
  state.phase = phase;
  state.phaseEnteredMillis = nowMillis;
}

std::uint32_t elapsedSince(std::uint32_t nowMillis, std::uint32_t sinceMillis) {
  // millis() の 32bit wrap をまたいでも正しく出るよう、符号なしの引き算で見る。
  return nowMillis - sinceMillis;
}

// 目標が決まっている状態で、天体を解いてサーボ指令を作る。
void solveForTarget(State& state, const Tick& tick, const astro::Observer& observer) {
  state.lastPosition =
      astro::computeTargetPosition(state.target, tick.unixSeconds, observer, tick.timeValid);
  if (!state.lastPosition.valid) {
    state.hasSolve = false;
    return;
  }

  pointing::SolveInput input;
  input.targetAzimuthDegrees = state.lastPosition.horizontal.azimuthDegrees;
  input.targetAltitudeDegrees = state.lastPosition.horizontal.altitudeDegrees;
  input.bodyHeadingDegrees = tick.bodyTrueHeadingDegrees;
  if (state.hasSolve) {
    input.currentYawDeciDegrees = state.lastSolve.command.yawDeciDegrees;
    input.currentPitchDeciDegrees = state.lastSolve.command.pitchDeciDegrees;
  }

  state.lastSolve = pointing::solve(input);
  state.hasSolve = true;
}

// スワイプ・クリックを処理する。ターゲットが変わったら true。
bool applyInput(State& state, const Tick& tick) {
  if (tick.input == Input::Click) {
    state.autoCycleEnabled = !state.autoCycleEnabled;
    state.lastTargetSwitchMillis = tick.nowMillis;
    return false;
  }

  const bool isSwipe = tick.input == Input::SwipeForward || tick.input == Input::SwipeBackward;
  if (!isSwipe) {
    return false;
  }

  state.target =
      tick.input == Input::SwipeForward ? nextTarget(state.target) : previousTarget(state.target);
  state.lastTargetSwitchMillis = tick.nowMillis;
  // 手動で選んだら自動巡回は止める (勝手に動かれると選んだ意味がない)
  state.autoCycleEnabled = false;
  return true;
}

} // namespace

const char* phaseName(Phase phase) {
  switch (phase) {
  case Phase::InitHardware:
    return "InitHardware";
  case Phase::ConnectWifi:
    return "ConnectWifi";
  case Phase::SyncTime:
    return "SyncTime";
  case Phase::Calibrating:
    return "Calibrating";
  case Phase::Idle:
    return "Idle";
  case Phase::ReturningToMeasurePose:
    return "ToMeasurePose";
  case Phase::Measuring:
    return "Measuring";
  case Phase::Pointing:
    return "Pointing";
  case Phase::Tracking:
    return "Tracking";
  case Phase::Error:
    break;
  }
  return "Error";
}

ServoIntent servoIntentFor(const State& state) {
  ServoIntent intent;

  // 測定に関わる局面では、首を必ず正面へ。これがノイズ対策の本体。
  const bool needsMeasurePose = state.phase == Phase::ReturningToMeasurePose ||
                                state.phase == Phase::Measuring ||
                                state.phase == Phase::Calibrating;
  if (needsMeasurePose) {
    intent.yawDeciDegrees = compass::kMeasurementYawDeci;
    intent.pitchDeciDegrees = pointing::kPitchLevelDeci;
    intent.shouldMove = true;
    return intent;
  }

  const bool canPoint = state.phase == Phase::Pointing || state.phase == Phase::Tracking;
  if (!canPoint || !state.hasSolve) {
    return intent;
  }

  intent.yawDeciDegrees = state.lastSolve.command.yawDeciDegrees;
  intent.pitchDeciDegrees = state.lastSolve.command.pitchDeciDegrees;
  // Pointing に入った直後は必ず動かす。Tracking 中は deadband を尊重する。
  intent.shouldMove = state.phase == Phase::Pointing || state.lastSolve.shouldMove;
  return intent;
}

astro::Target nextTarget(astro::Target current) {
  const auto count = static_cast<std::uint8_t>(astro::Target::kCount);
  const auto index = static_cast<std::uint8_t>(current);
  return static_cast<astro::Target>((index + 1) % count);
}

astro::Target previousTarget(astro::Target current) {
  const auto count = static_cast<std::uint8_t>(astro::Target::kCount);
  const auto index = static_cast<std::uint8_t>(current);
  return static_cast<astro::Target>((index + count - 1) % count);
}

void step(State& state, const Tick& tick, const Config& config, const astro::Observer& observer) {
  state.lastReject = tick.lastReject;

  // 入力はどの状態でも受け付ける。ターゲットが変われば測り直しから入る。
  const bool targetChanged = applyInput(state, tick);
  const bool isInteractive = state.phase == Phase::Idle || state.phase == Phase::Tracking ||
                             state.phase == Phase::Pointing;
  if (targetChanged && isInteractive) {
    enterPhase(state, Phase::ReturningToMeasurePose, tick.nowMillis);
    return;
  }

  switch (state.phase) {
  case Phase::InitHardware:
    enterPhase(state, Phase::ConnectWifi, tick.nowMillis);
    return;

  case Phase::ConnectWifi:
    // Wi-Fi の成否によらず先へ進む。真北は時計が無くても指せるので、
    // 時刻同期に失敗してもデバイスとして無価値にはならない。
    enterPhase(state, Phase::SyncTime, tick.nowMillis);
    return;

  case Phase::SyncTime:
    enterPhase(state, tick.calibrationValid ? Phase::Idle : Phase::Calibrating, tick.nowMillis);
    return;

  case Phase::Calibrating:
    // 8 の字回しが終わる (呼び出し側が calibrationValid を立てる) まで留まる。
    if (tick.calibrationValid) {
      enterPhase(state, Phase::Idle, tick.nowMillis);
    }
    return;

  case Phase::Idle:
    enterPhase(state, Phase::ReturningToMeasurePose, tick.nowMillis);
    return;

  case Phase::ReturningToMeasurePose: {
    // 首が正面に戻り、磁場が落ち着くまで待つ。ここを省くと首の角度による
    // バイアス (実測で最大 119 度) がそのまま方位に乗る。
    const bool settled =
        elapsedSince(tick.nowMillis, state.phaseEnteredMillis) >= config.measurePoseSettleMillis;
    if (settled) {
      enterPhase(state, Phase::Measuring, tick.nowMillis);
    }
    return;
  }

  case Phase::Measuring: {
    if (tick.measurementAccepted && tick.headingValid) {
      state.lastMeasureMillis = tick.nowMillis;
      solveForTarget(state, tick, observer);
      enterPhase(state, Phase::Pointing, tick.nowMillis);
      return;
    }
    const bool timedOut =
        elapsedSince(tick.nowMillis, state.phaseEnteredMillis) >= config.measureTimeoutMillis;
    if (timedOut) {
      // 測れなくても、以前の方位があるなら指しに行く。
      // 何も出さずに固まるより、古い情報でも動いている方が状態が読める。
      if (tick.headingValid) {
        solveForTarget(state, tick, observer);
        enterPhase(state, Phase::Pointing, tick.nowMillis);
        return;
      }
      enterPhase(state, Phase::Idle, tick.nowMillis);
    }
    return;
  }

  case Phase::Pointing: {
    const bool settled =
        tick.servoSettled &&
        elapsedSince(tick.nowMillis, state.phaseEnteredMillis) >= config.servoSettleMillis;
    if (settled) {
      enterPhase(state, Phase::Tracking, tick.nowMillis);
    }
    return;
  }

  case Phase::Tracking: {
    // 方位が無いまま追尾に入っていたら、すぐ測り直しへ戻る。
    //
    // Why not 再測定の周期を待つ: 方位が無効だと solveForTarget が解を作れず、
    // servoIntentFor も shouldMove を返さない。首が動かないので機体の動きも
    // 検出されず、周期が来るまで何もしないまま固まる。実機では首が -48 度の
    // まま Tracking に留まり続けた。
    if (!tick.headingValid) {
      enterPhase(state, Phase::ReturningToMeasurePose, tick.nowMillis);
      return;
    }

    // 機体ごと動かされたら方位が変わっているので測り直す。
    const bool bodyMoved = tick.gyroMagnitudeDegPerSec > config.bodyMovedGyroDegPerSec;
    if (bodyMoved) {
      enterPhase(state, Phase::ReturningToMeasurePose, tick.nowMillis);
      return;
    }

    const bool shouldCycle =
        state.autoCycleEnabled && elapsedSince(tick.nowMillis, state.lastTargetSwitchMillis) >=
                                      config.autoCycleIntervalMillis;
    if (shouldCycle) {
      state.target = nextTarget(state.target);
      state.lastTargetSwitchMillis = tick.nowMillis;
      enterPhase(state, Phase::ReturningToMeasurePose, tick.nowMillis);
      return;
    }

    const bool shouldRemeasure =
        elapsedSince(tick.nowMillis, state.lastMeasureMillis) >= config.remeasureIntervalMillis;
    if (shouldRemeasure) {
      enterPhase(state, Phase::ReturningToMeasurePose, tick.nowMillis);
      return;
    }

    // 天体は動き続けるので、方位はそのままでも指令を更新する。
    solveForTarget(state, tick, observer);
    return;
  }

  case Phase::Error:
    return;
  }
}

} // namespace app
