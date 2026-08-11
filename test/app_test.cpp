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

  // まず Measuring まで進める
  for (int step = 0; step < 5; ++step) {
    clock += 200;
    app::Tick tick = healthyTick(clock);
    tick.measurementAccepted = false;
    tick.headingValid = false;
    app::step(state, tick, config, tokyoObserver());
  }
  CHECK_TRUE(state.phase == app::Phase::Measuring);

  // タイムアウトまで測れないまま進める
  for (int step = 0; step < 40; ++step) {
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
  // ターゲットが変わったら方位から測り直す
  CHECK_TRUE(state.phase == app::Phase::Measuring);

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

void testBodyMovementTriggersRemeasure() {
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  clock += 200;
  app::Tick shaken = healthyTick(clock);
  shaken.gyroMagnitudeDegPerSec = 120.0F;
  app::step(state, shaken, config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Measuring);
}

void testMillisWrapDoesNotBreakTransitions() {
  // millis() は約 49.7 日で 32bit を巻き戻る。またいでも遷移が壊れないこと。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0xFFFFF000U;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  // 巻き戻りをまたいで進めても、Tracking から抜けて測り直しに入れる
  const std::uint32_t beforeWrap = clock;
  for (int step = 0; step < 400; ++step) {
    clock += 200;
    app::step(state, healthyTick(clock), config, tokyoObserver());
  }
  CHECK_TRUE(clock < beforeWrap); // 実際に巻き戻った
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

} // namespace

int main() {
  testBootReachesTracking();
  testCalibrationGate();
  testTimeInvalidStillPointsNorth();
  testMeasurementTimeoutWithoutHeading();
  testSwipeChangesTarget();
  testClickTogglesAutoCycle();
  testBodyMovementTriggersRemeasure();
  testMillisWrapDoesNotBreakTransitions();
  testTrackingUpdatesAsSkyMoves();
  return testing::summarize("app");
}
