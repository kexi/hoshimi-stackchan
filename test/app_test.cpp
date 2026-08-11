// app_core が保証すること:
//  - 起動から Idle まで、時刻同期やキャリブレーションの成否によらず前へ進む
//  - 測定が通れば Pointing → Tracking と進み、通らなければタイムアウトで戻る
//  - スワイプでターゲットが順送り・逆送りされ、自動巡回は止まる
//  - 機体が動かされたら測り直しに戻る
//  - millis() の 32bit wrap をまたいでも遷移が壊れない

#include "app/presentation.hpp"
#include "app/state.hpp"

#include "test_support.hpp"

#include <string_view>

namespace {

astro::Observer tokyoObserver() {
  astro::Observer observer;
  observer.latitudeDegrees = 35.681236;
  observer.longitudeEastDegrees = 139.767125;
  return observer;
}

// 2026-08-11 12:00 JST
constexpr std::int64_t kNoonJst = 1786417200;

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

std::size_t utf8CodePointCount(std::string_view text) {
  std::size_t count = 0;
  for (const char character : text) {
    const auto byte = static_cast<unsigned char>(character);
    const bool isContinuationByte = (byte & 0xC0U) == 0x80U;
    count += static_cast<std::size_t>(!isContinuationByte);
  }
  return count;
}

app::Tick measurementTick(std::uint32_t nowMillis) { return healthyTick(nowMillis); }

// 状態が変わらなくなるか、上限に達するまで進める。
void advanceUntil(app::State& state, app::Phase wanted, std::uint32_t& clock,
                  const app::Config& config) {
  for (int guard = 0; guard < 500 && state.phase != wanted; ++guard) {
    clock += 200;
    app::step(state, healthyTick(clock), config, tokyoObserver());
  }
}

// 正面の測定姿勢を使う経路で、指定した局面まで進める。
void advanceUsingMeasurePose(app::State& state, app::Phase wanted, std::uint32_t& clock,
                             const app::Config& config) {
  for (int guard = 0; guard < 500 && state.phase != wanted; ++guard) {
    clock += 200;
    app::step(state, measurementTick(clock), config, tokyoObserver());
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
  CHECK_TRUE(state.autoCycleEnabled);
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

void testDirectTargetSelection() {
  // Wi-Fiから特定対象を指定すると、順送りせず直接切り替わることを保証する。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  state.autoCycleEnabled = true;

  clock += 200;
  app::Tick selection = healthyTick(clock);
  selection.targetSelectionRequested = true;
  selection.requestedTarget = astro::Target::Mars;
  app::step(state, selection, config, tokyoObserver());

  CHECK_TRUE(state.target == astro::Target::Mars);
  CHECK_TRUE(!state.autoCycleEnabled);
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);
}

void testAutoCycleStartsEnabledAndClickToggles() {
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);

  CHECK_TRUE(state.autoCycleEnabled);

  // 起動後は自動巡回し、一定時間でターゲットが進むこと。
  const astro::Target before = state.target;
  for (int step = 0; step < 100; ++step) {
    clock += 200;
    app::step(state, healthyTick(clock), config, tokyoObserver());
  }
  CHECK_TRUE(state.target != before);

  clock += 200;
  app::Tick click = healthyTick(clock);
  click.input = app::Input::Click;
  app::step(state, click, config, tokyoObserver());
  CHECK_TRUE(!state.autoCycleEnabled);

  // もう一度タップすれば巡回を再開できること。
  clock += 200;
  click.nowMillis = clock;
  app::step(state, click, config, tokyoObserver());
  CHECK_TRUE(state.autoCycleEnabled);

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
  // 機体が動いたら古い方位を破棄し、正面・水平で測り直すこと。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  clock += 200;
  app::Tick shaken = healthyTick(clock);
  shaken.gyroMagnitudeDegPerSec = 120.0F;
  app::step(state, shaken, config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);
  CHECK_TRUE(!state.hasHeading);
  CHECK_TRUE(!state.hasSolve);
  CHECK_TRUE(state.forceMeasurementPose);
  CHECK_TRUE(state.headingResetRequested);
  CHECK_TRUE(state.bodyMotionCount == 1);

  const app::ServoIntent intent = app::servoIntentFor(state);
  CHECK_TRUE(intent.yawDeciDegrees == compass::kMeasurementYawDeci);
  CHECK_TRUE(intent.pitchDeciDegrees == pointing::kPitchLevelDeci);
  CHECK_TRUE(intent.shouldMove);
}

void testServoMotionDoesNotTriggerRemeasure() {
  // CoreS3のIMUは顔側にあるため、首を動かすだけでもジャイロが反応する。
  // サーボ稼働中と停止直後の慣性振動を本体移動と誤認せず、十分に静穏な
  // 時間が過ぎてからは実際の本体移動を検出することを保証する。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  clock += 200;
  app::Tick servoMotion = healthyTick(clock);
  servoMotion.servoSettled = false;
  servoMotion.gyroMagnitudeDegPerSec = 120.0F;
  app::step(state, servoMotion, config, tokyoObserver());

  CHECK_TRUE(state.phase == app::Phase::Tracking);
  CHECK_TRUE(state.hasSolve);

  clock += config.gyroAfterServoSettleMillis - 1;
  app::Tick inertia = healthyTick(clock);
  inertia.servoSettled = true;
  inertia.gyroMagnitudeDegPerSec = 120.0F;
  app::step(state, inertia, config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  clock += 1;
  app::Tick bodyMotion = healthyTick(clock);
  bodyMotion.servoSettled = true;
  bodyMotion.gyroMagnitudeDegPerSec = 120.0F;
  app::step(state, bodyMotion, config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);
}

void testMeasurementPoseCanPointNorth() {
  // 正面で方位を採れば、追加の首角度モデルなしで真北を指せること。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  for (int step = 0; step < 80 && state.phase != app::Phase::Tracking; ++step) {
    clock += 200;
    app::step(state, measurementTick(clock), config, tokyoObserver());
  }
  CHECK_TRUE(state.phase == app::Phase::Tracking);
  CHECK_TRUE(state.target == astro::Target::North);
  CHECK_TRUE(state.hasSolve);
}

void testMovementForcesFreshMeasurement() {
  // 本体移動後は古い方位を使わず基準姿勢へ戻ること。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  advanceUntil(state, app::Phase::Tracking, clock, config);
  CHECK_TRUE(state.phase == app::Phase::Tracking);

  clock += 200;
  app::Tick moved = healthyTick(clock);
  moved.gyroMagnitudeDegPerSec = 20.0F;
  app::step(state, moved, config, tokyoObserver());

  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);
  CHECK_TRUE(state.forceMeasurementPose);
  CHECK_TRUE(state.headingResetRequested);
  CHECK_TRUE(!state.hasHeading);

  const app::ServoIntent intent = app::servoIntentFor(state);
  CHECK_TRUE(intent.yawDeciDegrees == compass::kMeasurementYawDeci);
  CHECK_TRUE(intent.pitchDeciDegrees == pointing::kPitchLevelDeci);

  clock += 200;
  app::Tick waiting = healthyTick(clock);
  app::step(state, waiting, config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);
  CHECK_TRUE(state.forceMeasurementPose);
  CHECK_TRUE(!state.headingResetRequested);

  clock += config.measurePoseSettleMillis;
  app::Tick settled = healthyTick(clock);
  app::step(state, settled, config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Measuring);

  clock += 200;
  app::Tick measured = healthyTick(clock);
  measured.bodyTrueHeadingDegrees = 123.0F;
  app::step(state, measured, config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Pointing);
  CHECK_TRUE(state.hasHeading);
  CHECK_NEAR(state.bodyHeadingDegrees, 123.0F, 0.001F);
  CHECK_TRUE(!state.forceMeasurementPose);
  CHECK_TRUE(state.hasSolve);
}

void testMeasurementAlwaysUsesMeasurePose() {
  // 横向きではpitchと磁場強度も変わるので、正面・水平姿勢を使うこと。
  app::State measuring;
  measuring.phase = app::Phase::Measuring;
  measuring.hasSolve = true;
  measuring.lastSolve.command.yawDeciDegrees = 700;
  measuring.lastSolve.command.pitchDeciDegrees = 600;
  measuring.lastSolve.shouldMove = true;

  const app::ServoIntent intent = app::servoIntentFor(measuring);
  CHECK_TRUE(intent.yawDeciDegrees == compass::kMeasurementYawDeci);
  CHECK_TRUE(intent.pitchDeciDegrees == pointing::kPitchLevelDeci);
  CHECK_TRUE(intent.shouldMove);

  measuring.phase = app::Phase::ReturningToMeasurePose;

  const app::ServoIntent returning = app::servoIntentFor(measuring);
  CHECK_TRUE(returning.yawDeciDegrees == compass::kMeasurementYawDeci);
  CHECK_TRUE(returning.pitchDeciDegrees == pointing::kPitchLevelDeci);
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
  advanceUsingMeasurePose(state, app::Phase::ReturningToMeasurePose, clock, config);

  bool sawMeasurePose = false;
  for (int step = 0; step < 60; ++step) {
    clock += 200;
    app::step(state, measurementTick(clock), config, tokyoObserver());
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
  advanceUsingMeasurePose(state, app::Phase::ReturningToMeasurePose, clock, config);
  const std::uint32_t entered = clock;

  // settle 未満では Measuring に進まない
  clock += 400;
  app::step(state, measurementTick(clock), config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::ReturningToMeasurePose);

  // settle を超えたら進む
  clock = entered + 1200;
  app::step(state, measurementTick(clock), config, tokyoObserver());
  CHECK_TRUE(state.phase == app::Phase::Measuring);
}

void testFaceSpeechNamesTargetAndFitsBalloon() {
  // 全対象の案内が、地平線下と可動域外のどちらでも対象名を明示し、
  // Avatarの吹き出しに収まる6文字以内であることを保証する。
  for (std::uint8_t index = 0; index < static_cast<std::uint8_t>(astro::Target::kCount); ++index) {
    app::State state;
    state.phase = app::Phase::Tracking;
    state.target = static_cast<astro::Target>(index);
    state.lastPosition.valid = true;
    state.lastPosition.aboveHorizon = false;
    state.lastSolve.command.clampedYaw = true;
    state.lastSolve.command.clampedPitch = true;

    const std::string_view targetName = app::targetNameJapanese(state.target);
    const app::FacePresentation below = app::facePresentationFor(state, 1.0F);
    const std::string_view belowSpeech = below.speech.data();
    CHECK_TRUE(below.mood == app::FaceMood::Sleepy);
    CHECK_TRUE(belowSpeech.rfind(targetName, 0) == 0);
    CHECK_TRUE(belowSpeech.find("は地平下") != std::string_view::npos);
    CHECK_TRUE(utf8CodePointCount(belowSpeech) <= 6);

    state.lastPosition.aboveHorizon = true;
    const app::FacePresentation behind = app::facePresentationFor(state, 1.0F);
    const std::string_view behindSpeech = behind.speech.data();
    CHECK_TRUE(behind.mood == app::FaceMood::Doubt);
    CHECK_TRUE(behindSpeech.rfind(targetName, 0) == 0);
    CHECK_TRUE(behindSpeech.find("はうしろ") != std::string_view::npos);
    CHECK_TRUE(utf8CodePointCount(behindSpeech) <= 6);

    state.lastSolve.command.clampedYaw = false;
    const app::FacePresentation above = app::facePresentationFor(state, 1.0F);
    const std::string_view aboveSpeech = above.speech.data();
    CHECK_TRUE(above.mood == app::FaceMood::Doubt);
    CHECK_TRUE(aboveSpeech.rfind(targetName, 0) == 0);
    CHECK_TRUE(aboveSpeech.find("は上すぎ") != std::string_view::npos);
    CHECK_TRUE(utf8CodePointCount(aboveSpeech) <= 6);

    state.lastSolve.command.clampedPitch = false;
    const app::FacePresentation pointing = app::facePresentationFor(state, 1.0F);
    CHECK_TRUE(pointing.mood == app::FaceMood::Happy);
    CHECK_TRUE(utf8CodePointCount(pointing.speech.data()) <= 6);
  }
}

} // namespace

int main() {
  testMeasurementPoseIsCommanded();
  testPointingDoesNotReissueCommandForever();
  testMeasurePoseSettleOutlastsServoTravel();
  testMeasurePoseSettleIsRespected();
  testFaceSpeechNamesTargetAndFitsBalloon();
  testBootReachesTracking();
  testCalibrationGate();
  testTimeInvalidStillPointsNorth();
  testMeasurementTimeoutWithoutHeading();
  testSwipeChangesTarget();
  testDirectTargetSelection();
  testAutoCycleStartsEnabledAndClickToggles();
  testTrackingStaysWhileNeckIsAway();
  testTrackingEscapesWhenHeadingWasNeverTaken();
  testBodyMovementTriggersRemeasure();
  testServoMotionDoesNotTriggerRemeasure();
  testMeasurementPoseCanPointNorth();
  testMovementForcesFreshMeasurement();
  testMeasurementAlwaysUsesMeasurePose();
  testMillisWrapDoesNotBreakTransitions();
  testTrackingUpdatesAsSkyMoves();
  return testing::summarize("app");
}
