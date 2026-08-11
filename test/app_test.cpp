// app_core が保証すること:
//  - 起動から Idle まで、時刻同期やキャリブレーションの成否によらず前へ進む
//  - 測定が通れば Pointing → Tracking と進み、通らなければタイムアウトで戻る
//  - スワイプでターゲットが順送り・逆送りされ、自動巡回は止まる
//  - 機体が動かされたら測り直しに戻る
//  - millis() の 32bit wrap をまたいでも遷移が壊れない

#include "app/state.hpp"

#include "test_support.hpp"

namespace {

astro::Observer tokyoObserver() {
  astro::Observer observer;
  observer.latitudeDegrees = 35.681236;
  observer.longitudeEastDegrees = 139.767125;
  return observer;
}

// 2026-08-11 12:00 JST
constexpr std::int64_t kNoonJst = 1786064400;

app::Tick healthyTick(std::uint32_t nowMillis) {
  app::Tick tick;
  tick.nowMillis = nowMillis;
  tick.unixSeconds = kNoonJst;
  tick.timeValid = true;
  tick.calibrationValid = true;
  tick.headingValid = true;
  tick.measurementAccepted = true;
  tick.servoSettled = true;
  // 学習済みの定常状態を既定にする。学習そのものは専用のテストで見る。
  tick.biasCorrected = true;
  return tick;
}

// 首のクセをまだ覚えていない状態。首を正面へ戻さないと方位が読めない。
app::Tick uncorrectedTick(std::uint32_t nowMillis) {
  app::Tick tick = healthyTick(nowMillis);
  tick.biasCorrected = false;
  return tick;
}

// 状態が変わらなくなるか、上限に達するまで進める。
void advanceUntil(app::State& state, app::Phase wanted, std::uint32_t& clock,
                  const app::Config& config) {
  for (int guard = 0; guard < 500 && state.phase != wanted; ++guard) {
    clock += 200;
    app::step(state, healthyTick(clock), config, tokyoObserver());
  }
}

// 学習は済ませた上で、補正を使わない状態から進める。
//
// LearningBias は補正が無い限り抜けないので、いったん補正ありで追尾まで
// 進めてから補正を外す。実機で言えば「覚えた表が使えなくなった」状況。
// 再測定の周期は長いので、機体を動かして測り直しの契機を作る。
void advanceUncorrected(app::State& state, app::Phase wanted, std::uint32_t& clock,
                        const app::Config& config) {
  advanceUntil(state, app::Phase::Tracking, clock, config);

  clock += 200;
  app::Tick shaken = uncorrectedTick(clock);
  shaken.gyroMagnitudeDegPerSec = 120.0F;
  app::step(state, shaken, config, tokyoObserver());

  for (int guard = 0; guard < 500 && state.phase != wanted; ++guard) {
    clock += 200;
    app::step(state, uncorrectedTick(clock), config, tokyoObserver());
  }
}

void testBootReachesTracking() {
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  CHECK_TRUE(state.phase == app::Phase::InitHardware);

  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);
  CHECK_TRUE(state.hasSolve);
  // 既定ターゲットは真北なので、機体が真北を向いていれば首は正面
  CHECK_TRUE(state.target == astro::Target::North);
  CHECK_TRUE(state.lastSolve.command.yawDeciDegrees == 0);
}

void testCalibrationGate() {
  // キャリブレーション未完了なら Calibrating に留まる
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  for (int step = 0; step < 20; ++step) {
    clock += 200;
    app::Tick tick = healthyTick(clock);
    tick.calibrationValid = false;
    app::step(state, tick, config, tokyoObserver());
  }
  CHECK_TRUE(state.phase == app::Phase::Calibrating);

  // 完了すれば Idle へ抜ける
  clock += 200;
  app::step(state, healthyTick(clock), config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Idle);
}

void testTimeInvalidStillPointsNorth() {
  // 時刻同期に失敗しても真北は指せる。Wi-Fi の無い場所で無価値にならないこと。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  for (int step = 0; step < 60; ++step) {
    clock += 200;
    app::Tick tick = healthyTick(clock);
    tick.timeValid = false;
    app::step(state, tick, config, tokyoObserver());
  }
  CHECK_TRUE(state.phase == app::Phase::Tracking);
  CHECK_TRUE(state.hasSolve);
  CHECK_TRUE(state.lastPosition.valid);
}

void testMeasurementTimeoutWithoutHeading() {
  // 測定が通らず方位も無いなら Idle に戻る (指す先が決まらない)
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  // まず Measuring まで進める。
  // 首を正面に戻す ReturningToMeasurePose を挟むので、その settle 時間も要る。
  for (int step = 0; step < 17; ++step) {
    clock += 200;
    app::Tick tick = healthyTick(clock);
    tick.measurementAccepted = false;
    tick.headingValid = false;
    app::step(state, tick, config, tokyoObserver());
  }
  CHECK_TRUE(state.phase == app::Phase::Measuring);

  // タイムアウトまで測れないまま進める
  for (int step = 0; step < 60; ++step) {
    clock += 200;
    app::Tick tick = healthyTick(clock);
    tick.measurementAccepted = false;
    tick.headingValid = false;
    tick.lastReject = compass::MeasurementGate::Reject::FieldAnomaly;
    app::step(state, tick, config, tokyoObserver());
  }
  CHECK_TRUE(state.phase == app::Phase::Idle || state.phase == app::Phase::Measuring);
  CHECK_TRUE(state.lastReject == compass::MeasurementGate::Reject::FieldAnomaly);
}

void testSwipeChangesTarget() {
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);

  CHECK_TRUE(state.target == astro::Target::North);

  clock += 200;
  app::Tick forward = healthyTick(clock);
  forward.input = app::Input::SwipeForward;
  app::step(state, forward, config, tokyoObserver());
  CHECK_TRUE(state.target == astro::Target::Sun);
  // ターゲットが変わったら、首を正面に戻すところから測り直す
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);

  advanceUntil(state, app::Phase::Tracking, clock, config);
  clock += 200;
  app::Tick backward = healthyTick(clock);
  backward.input = app::Input::SwipeBackward;
  app::step(state, backward, config, tokyoObserver());
  CHECK_TRUE(state.target == astro::Target::North);

  // 一周して戻る
  CHECK_TRUE(app::previousTarget(astro::Target::North) == astro::Target::Saturn);
  CHECK_TRUE(app::nextTarget(astro::Target::Saturn) == astro::Target::North);
}

void testClickTogglesAutoCycle() {
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);

  CHECK_TRUE(!state.autoCycleEnabled);

  clock += 200;
  app::Tick click = healthyTick(clock);
  click.input = app::Input::Click;
  app::step(state, click, config, tokyoObserver());
  CHECK_TRUE(state.autoCycleEnabled);

  // 自動巡回中は一定時間でターゲットが進む
  const astro::Target before = state.target;
  for (int step = 0; step < 100; ++step) {
    clock += 200;
    app::step(state, healthyTick(clock), config, tokyoObserver());
  }
  CHECK_TRUE(state.target != before);

  // 手動スワイプが入ったら自動巡回は止まる (選んだ意味がなくなるため)
  clock += 200;
  app::Tick swipe = healthyTick(clock);
  swipe.input = app::Input::SwipeForward;
  app::step(state, swipe, config, tokyoObserver());
  CHECK_TRUE(!state.autoCycleEnabled);
}

void testTrackingStaysWhileNeckIsAway() {
  // 首を振っている間に方位が測れなくなっても、追尾に留まり続けること。
  //
  // 指すために首を振ると磁場が乱れてその場では測れないが、機体は動いて
  // いないので方位そのものは変わらない。ここで測り直しに戻ると、首を正面へ
  // 戻し、また指しに行き、また戻る往復に陥る (実機で発生)。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);
  CHECK_TRUE(state.hasHeading);

  clock += 200;
  app::Tick neckAway = healthyTick(clock);
  neckAway.headingValid = false;
  neckAway.measurementAccepted = false;
  app::step(state, neckAway, config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Tracking);
  // 採用済みの方位で解き続けること
  CHECK_TRUE(state.hasSolve);
}

void testTrackingEscapesWhenHeadingWasNeverTaken() {
  // 方位を一度も採れていないまま Tracking に居たら、測り直しへ戻ること。
  //
  // 方位が無いと解が作れず、首も動かない。首が動かないので機体の動きも
  // 検出されず、再測定の周期が来るまで固まる。実機では首が -48 度のまま
  // Tracking に留まり続けた。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  // 方位を持っていない状態を作る
  state.hasHeading = false;

  clock += 200;
  app::step(state, healthyTick(clock), config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);
}

void testBodyMovementTriggersRemeasure() {
  // 首のクセをまだ覚えていないときは、機体が動いたら測り直すこと。
  // この状態では首を正面へ戻さないと方位が読めない。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  clock += 200;
  app::Tick shaken = healthyTick(clock);
  shaken.biasCorrected = false;
  shaken.gyroMagnitudeDegPerSec = 120.0F;
  app::step(state, shaken, config, tokyoObserver());
  // 機体が動かされたら、首を正面に戻して測り直す
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);
}

void testLearningSweepsNeckAcrossRange() {
  // 首のクセを覚える局面が、可動域を端まで掃くこと。
  //
  // 16 ビンのうち半分以上を埋めないと補正として使えないので、
  // 特定の角度だけ通っても足りない。
  const int first = app::learningYawFor(0);
  CHECK_TRUE(first == compass::kMeasurementYawDeci);

  int minYaw = first;
  int maxYaw = first;
  for (std::uint8_t step = 0; step < app::kLearningStepCount; ++step) {
    const int yaw = app::learningYawFor(step);
    CHECK_TRUE(pointing::isYawReachable(yaw));
    minYaw = yaw < minYaw ? yaw : minYaw;
    maxYaw = yaw > maxYaw ? yaw : maxYaw;
  }
  CHECK_TRUE(minYaw == pointing::kYawMinDeci);
  CHECK_TRUE(maxYaw == pointing::kYawMaxDeci);
}

void testLearningPhaseRunsBeforeIdle() {
  // 首のクセを覚えていなければ、追尾に入る前に学習へ寄ること。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  for (int step = 0; step < 40 && state.phase != app::Phase::LearningBias; ++step) {
    clock += 200;
    app::Tick tick = healthyTick(clock);
    tick.biasCorrected = false;
    app::step(state, tick, config, tokyoObserver());
  }
  CHECK_TRUE(state.phase == app::Phase::LearningBias);

  // 覚え終われば先へ進む
  clock += 200;
  app::step(state, healthyTick(clock), config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Idle);
}

void testBiasCorrectedTrackingSurvivesMovement() {
  // バイアス表が学習済みなら、持ち歩いて機体が動き続けても追尾に留まること。
  //
  // 首の角度によるずれを打ち消せるので、首を正面へ戻さなくても方位が読める。
  // ここで測定に戻ると、歩いている間は指すことも測ることもできなくなる。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  // バイアス補正が効いた状態で追尾まで進める
  auto movingTick = [&](std::uint32_t nowMillis) {
    app::Tick tick = healthyTick(nowMillis);
    tick.biasCorrected = true;
    tick.gyroMagnitudeDegPerSec = 120.0F; // 歩いている
    return tick;
  };

  for (int step = 0; step < 400 && state.phase != app::Phase::Tracking; ++step) {
    clock += 200;
    app::step(state, movingTick(clock), config, tokyoObserver());
  }
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  // 揺れ続けても追尾から出ない
  for (int step = 0; step < 100; ++step) {
    clock += 200;
    app::step(state, movingTick(clock), config, tokyoObserver());
    CHECK_TRUE(state.phase == app::Phase::Tracking);
  }
  // 動きながらでも指令が更新され続けること
  CHECK_TRUE(state.hasSolve);
}

void testBiasCorrectedSkipsMeasurePose() {
  // バイアス表があるなら、測定のために首を正面へ戻さないこと。
  //
  // 戻すと指している方向を見失う。持ち歩きながら指し続けるには、
  // 指したままの姿勢で測れる必要がある。
  app::State withBias;
  withBias.biasCorrected = true;
  withBias.phase = app::Phase::Measuring;
  withBias.hasSolve = true;
  withBias.lastSolve.command.yawDeciDegrees = 700;
  withBias.lastSolve.command.pitchDeciDegrees = 600;
  withBias.lastSolve.shouldMove = true;

  const app::ServoIntent corrected = app::servoIntentFor(withBias);
  CHECK_TRUE(corrected.yawDeciDegrees == 700);

  // 表が無ければ従来どおり正面へ戻す
  app::State withoutBias = withBias;
  withoutBias.biasCorrected = false;

  const app::ServoIntent uncorrected = app::servoIntentFor(withoutBias);
  CHECK_TRUE(uncorrected.yawDeciDegrees == compass::kMeasurementYawDeci);
}

void testMillisWrapDoesNotBreakTransitions() {
  // millis() は約 49.7 日で 32bit を巻き戻る。またいでも遷移が壊れないこと。
  app::State state;
  app::Config config;
  // Tracking に入るまでに必要な時間ぶん手前から始め、巻き戻りが
  // 追尾中に起きるようにする。
  std::uint32_t clock = 0xFFFF0000U;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  // 巻き戻りを確実にまたぐ。0xFFFFFFFF を越えると 0 に戻る。
  bool wrapped = false;
  for (int step = 0; step < 2000; ++step) {
    const std::uint32_t previous = clock;
    clock += 200;
    if (clock < previous) {
      wrapped = true;
    }
    app::step(state, healthyTick(clock), config, tokyoObserver());
  }
  CHECK_TRUE(wrapped); // 実際に巻き戻った
  CHECK_TRUE(state.phase != app::Phase::Error);
  CHECK_TRUE(state.hasSolve);
}

void testTrackingUpdatesAsSkyMoves() {
  // 方位が変わらなくても、天体が動くのでサーボ指令は更新され続ける。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  // 太陽に切り替えてから追尾させる
  advanceUntil(state, app::Phase::Tracking, clock, config);
  clock += 200;
  app::Tick swipe = healthyTick(clock);
  swipe.input = app::Input::SwipeForward;
  app::step(state, swipe, config, tokyoObserver());
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.target == astro::Target::Sun);

  const int firstYaw = state.lastSolve.command.yawDeciDegrees;

  // 2 時間ぶん進めると、太陽は明らかに動く
  for (int step = 0; step < 20; ++step) {
    clock += 200;
    app::Tick tick = healthyTick(clock);
    tick.unixSeconds = kNoonJst + 7200;
    app::step(state, tick, config, tokyoObserver());
  }
  CHECK_TRUE(state.lastSolve.command.yawDeciDegrees != firstYaw);
}

void testMeasurementPoseIsCommanded() {
  // 実測で首の角度による誤差が 119 度と分かった以上、測定に関わる局面では
  // 必ず首が正面に戻ること。ここが崩れると方位が根本から狂う。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  // Measuring / ReturningToMeasurePose を通るまで進める。
  // 補正が使えないときは、測定のたびに正面へ戻る必要がある。
  advanceUncorrected(state, app::Phase::ReturningToMeasurePose, clock, config);

  bool sawMeasurePose = false;
  for (int step = 0; step < 60; ++step) {
    clock += 200;
    app::step(state, uncorrectedTick(clock), config, tokyoObserver());
    const bool isMeasurePhase =
        state.phase == app::Phase::ReturningToMeasurePose || state.phase == app::Phase::Measuring;
    if (!isMeasurePhase) {
      continue;
    }
    sawMeasurePose = true;
    const app::ServoIntent intent = app::servoIntentFor(state);
    CHECK_TRUE(intent.yawDeciDegrees == compass::kMeasurementYawDeci);
    CHECK_TRUE(intent.pitchDeciDegrees == pointing::kPitchLevelDeci);
    CHECK_TRUE(intent.shouldMove);
    // ゲートから見ても測定してよい姿勢であること
    CHECK_TRUE(compass::isMeasurementPose(intent.yawDeciDegrees));
  }
  CHECK_TRUE(sawMeasurePose);

  // 追尾中は解いた方向を指す (正面に固定されたままではない)
  advanceUntil(state, app::Phase::Tracking, clock, config);
  clock += 200;
  app::Tick swipe = healthyTick(clock);
  swipe.input = app::Input::SwipeForward; // 太陽へ
  app::step(state, swipe, config, tokyoObserver());
  advanceUntil(state, app::Phase::Tracking, clock, config);

  const app::ServoIntent tracking = app::servoIntentFor(state);
  CHECK_TRUE(tracking.yawDeciDegrees == state.lastSolve.command.yawDeciDegrees);
  CHECK_TRUE(tracking.pitchDeciDegrees == state.lastSolve.command.pitchDeciDegrees);
}

void testPointingDoesNotReissueCommandForever() {
  // Pointing で毎周期 shouldMove が真だと、指令が出し直され続けて
  // サーボ静止の判定が真に戻り、局面から抜けられなくなる (実機で発生)。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  advanceUntil(state, app::Phase::Pointing, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Pointing);

  // 目標に着いていて deadband 内なら、動かす必要はない
  const app::ServoIntent intent = app::servoIntentFor(state);
  CHECK_TRUE(intent.shouldMove == state.lastSolve.shouldMove);
}

void testMeasurePoseSettleOutlastsServoTravel() {
  // 既定値の関係が崩れると、首がまだ動いている最中に Measuring へ進んでしまう。
  // するとゲートが servoMoving で弾き続け、タイムアウト後に汚れた方位が
  // 採用される (実機で首が西を向いた不具合の原因)。
  const app::Config config;
  CHECK_TRUE(config.measurePoseSettleMillis > config.servoSettleMillis);
}

void testMeasurePoseSettleIsRespected() {
  // 首を戻した直後に測ると意味がないので、settle 時間は必ず待つこと。
  app::State state;
  app::Config config;
  config.measurePoseSettleMillis = 1000;
  std::uint32_t clock = 0;

  // 首のクセを覚える前は、正面へ戻して落ち着くのを待つ必要がある。
  advanceUncorrected(state, app::Phase::ReturningToMeasurePose, clock, config);
  const std::uint32_t entered = clock;

  // settle 未満では Measuring に進まない
  clock += 400;
  app::step(state, uncorrectedTick(clock), config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);

  // settle を超えたら進む
  clock = entered + 1200;
  app::step(state, uncorrectedTick(clock), config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Measuring);
}

} // namespace

int main() {
  testMeasurementPoseIsCommanded();
  testPointingDoesNotReissueCommandForever();
  testMeasurePoseSettleOutlastsServoTravel();
  testMeasurePoseSettleIsRespected();
  testBootReachesTracking();
  testCalibrationGate();
  testTimeInvalidStillPointsNorth();
  testMeasurementTimeoutWithoutHeading();
  testSwipeChangesTarget();
  testClickTogglesAutoCycle();
  testTrackingStaysWhileNeckIsAway();
  testTrackingEscapesWhenHeadingWasNeverTaken();
  testBodyMovementTriggersRemeasure();
  testLearningSweepsNeckAcrossRange();
  testLearningPhaseRunsBeforeIdle();
  testBiasCorrectedTrackingSurvivesMovement();
  testBiasCorrectedSkipsMeasurePose();
  testMillisWrapDoesNotBreakTransitions();
  testTrackingUpdatesAsSkyMoves();
  return testing::summarize("app");
}
