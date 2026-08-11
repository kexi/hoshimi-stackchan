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
  // 採用済みの方位を使う。首を振っている間 tick 側は無効になるが、機体は
  // 動いていないので方位は変わらない。
  input.bodyHeadingDegrees = state.bodyHeadingDegrees;
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

int learningYawFor(std::uint8_t step) {
  // まず正面。ここで測った方位が、他の角度のずれを測るときの基準になる。
  if (step == 0 || step >= kLearningStepCount) {
    return compass::kMeasurementYawDeci;
  }
  // 残りで可動域 (-1280..1280) を等分に掃く。端まで舐めれば 16 ビンのうち
  // 半分以上が埋まる。
  constexpr int kSpan = pointing::kYawMaxDeci - pointing::kYawMinDeci;
  const int sweepIndex = static_cast<int>(step) - 1;
  const int sweepCount = static_cast<int>(kLearningStepCount) - 2;
  return pointing::kYawMinDeci + kSpan * sweepIndex / sweepCount;
}

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
  case Phase::LearningBias:
    return "LearningBias";
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

  // 学習中は首を順に振る。各角度で磁気を測り、正面との差を覚える。
  if (state.phase == Phase::LearningBias) {
    intent.yawDeciDegrees = learningYawFor(state.learningStep);
    intent.pitchDeciDegrees = pointing::kPitchLevelDeci;
    intent.shouldMove = true;
    return intent;
  }

  // 測定に関わる局面では、首を必ず正面へ。これがノイズ対策の本体。
  //
  // ただしバイアス表が学習済みなら、首の角度によるずれは打ち消せるので
  // 戻す必要がない。持ち歩きながら指し続けるにはこれが要る。
  // キャリブレーション中は表の有無によらず正面で固定する (回転する円を
  // 描くのに首が動いていると条件が変わってしまう)。
  const bool measuringWithoutBias =
      !state.biasCorrected &&
      (state.phase == Phase::ReturningToMeasurePose || state.phase == Phase::Measuring);
  const bool needsMeasurePose = measuringWithoutBias || state.phase == Phase::Calibrating;
  if (needsMeasurePose) {
    intent.yawDeciDegrees = compass::kMeasurementYawDeci;
    intent.pitchDeciDegrees = pointing::kPitchLevelDeci;
    intent.shouldMove = true;
    return intent;
  }

  // バイアス表があるなら測定中も指したままでよいので、その局面も含める。
  const bool canPoint = state.phase == Phase::Pointing || state.phase == Phase::Tracking ||
                        (state.biasCorrected && (state.phase == Phase::Measuring ||
                                                 state.phase == Phase::ReturningToMeasurePose));
  if (!canPoint || !state.hasSolve) {
    return intent;
  }

  intent.yawDeciDegrees = state.lastSolve.command.yawDeciDegrees;
  intent.pitchDeciDegrees = state.lastSolve.command.pitchDeciDegrees;
  // Pointing で無条件に真を返さない。
  //
  // Why not Pointing なら常に動かす: 呼び出し側は毎周期この関数を呼ぶ。
  // 常に真だと指令が出し直され続け、サーボ静止の判定が真に戻り続けて
  // Pointing から抜けられなくなる (実機で発生)。指令は局面に入った時点で
  // 出ているので、あとは deadband の判断に任せる。
  intent.shouldMove = state.lastSolve.shouldMove;
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
  state.biasCorrected = tick.biasCorrected;

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
    if (!tick.calibrationValid) {
      enterPhase(state, Phase::Calibrating, tick.nowMillis);
      return;
    }
    // キャリブレーション済みでも、首のクセを覚えていなければ先に学習する。
    state.learningStep = 0;
    enterPhase(state, tick.biasCorrected ? Phase::Idle : Phase::LearningBias, tick.nowMillis);
    return;

  case Phase::Calibrating:
    // 水平回しが終わる (呼び出し側が calibrationValid を立てる) まで留まる。
    if (tick.calibrationValid) {
      // 続けて首のクセを覚える。これが済むと首を正面へ戻さずに測れるので、
      // 持ち歩きながら指し続けられる。
      state.learningStep = 0;
      enterPhase(state, tick.biasCorrected ? Phase::Idle : Phase::LearningBias, tick.nowMillis);
    }
    return;

  case Phase::LearningBias: {
    // 学習が足りたら抜ける。呼び出し側が各角度で観測を積む。
    if (tick.biasCorrected) {
      enterPhase(state, Phase::Idle, tick.nowMillis);
      return;
    }

    // 首が目標へ着いて磁場が落ち着くまで待ち、次の角度へ進む。
    const bool settled =
        elapsedSince(tick.nowMillis, state.phaseEnteredMillis) >= config.measurePoseSettleMillis;
    if (!settled) {
      return;
    }
    if (state.learningStep + 1 < kLearningStepCount) {
      ++state.learningStep;
      state.phaseEnteredMillis = tick.nowMillis;
      return;
    }

    // 一周しても足りなければ、もう一周する。実機では角度によって
    // 磁場が安定せず、観測が採れないビンが出る。
    state.learningStep = 0;
    state.phaseEnteredMillis = tick.nowMillis;
    return;
  }

  case Phase::Idle:
    enterPhase(state, Phase::ReturningToMeasurePose, tick.nowMillis);
    return;

  case Phase::ReturningToMeasurePose: {
    // バイアス表があるなら首を戻す必要がない。待たずに測りに行く。
    // 持ち歩きながら指し続けるには、ここで足を止めていられない。
    if (state.biasCorrected) {
      enterPhase(state, Phase::Measuring, tick.nowMillis);
      return;
    }

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
      state.bodyHeadingDegrees = tick.bodyTrueHeadingDegrees;
      state.hasHeading = true;
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
      if (state.hasHeading) {
        solveForTarget(state, tick, observer);
        enterPhase(state, Phase::Pointing, tick.nowMillis);
        return;
      }
      enterPhase(state, Phase::Idle, tick.nowMillis);
    }
    return;
  }

  case Phase::Pointing: {
    // 指令を出してから一定時間経てば追尾へ移る。
    //
    // Why not サーボの静止を待つ: 指した先が可動域の端だと、サーボは目標に
    // 到達できず微振動を続ける。静止を条件にすると永久に抜けられない
    // (実機で発生)。時間だけで区切り、実際に指せたかは UI が clamped で示す。
    const bool waited =
        elapsedSince(tick.nowMillis, state.phaseEnteredMillis) >= config.servoSettleMillis;
    if (waited) {
      enterPhase(state, Phase::Tracking, tick.nowMillis);
    }
    return;
  }

  case Phase::Tracking: {
    // 方位を一度も採れていないなら、すぐ測り直しへ戻る。
    //
    // Why not tick.headingValid を見る: 指すために首を振ると磁場が乱れて
    // その場では測れなくなるが、機体は動いていないので方位は変わらない。
    // tick 側を条件にすると、指した瞬間に「方位を失った」と判断して測定へ戻り、
    // 首を正面に戻し、また指しに行く往復に陥る (実機で発生)。
    //
    // Why not 何も見ない: 方位が一度も無いと solveForTarget が解を作れず、
    // 首が動かないので機体の動きも検出されず、周期が来るまで固まる。
    if (!state.hasHeading) {
      enterPhase(state, Phase::ReturningToMeasurePose, tick.nowMillis);
      return;
    }

    // 機体ごと動かされたら方位が変わっているので測り直す。
    //
    // バイアス表があるなら戻らない。首を振ったままでも方位が読めるので、
    // 追尾に留まったまま更新できる。持ち歩いている間は常に動いているので、
    // ここで測定へ戻すと指すことも測ることもできなくなる。
    const bool bodyMoved = tick.gyroMagnitudeDegPerSec > config.bodyMovedGyroDegPerSec;
    if (bodyMoved && !state.biasCorrected) {
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

    // バイアス表があるなら、追尾しながら方位も更新し続ける。
    // 持ち歩いて向きが変わっても、首が指し続けるために要る。
    if (state.biasCorrected && tick.headingValid) {
      state.bodyHeadingDegrees = tick.bodyTrueHeadingDegrees;
      state.lastMeasureMillis = tick.nowMillis;
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
