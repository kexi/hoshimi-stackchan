// 段階 10 が保証すること: スワイプで天体が切り替わり、首がその天体を指す。
//
// 実機のタッチセンサから首の角度までを、ファームと同じ経路で通す。
// 実機で確認できるのは「スワイプしたら首が動いた」という事実だけだが、
// ここでは指した方向が天体計算と一致するところまで検証できる。

#include "app/state.hpp"
#include "compass/declination.hpp"

#include "test_support.hpp"

#include <cmath>

namespace {

constexpr double kTokyoLatitude = 35.681236;
constexpr double kTokyoLongitudeEast = 139.767125;
// 2026-08-11 12:00 JST。この時刻の天体配置で検証する。
constexpr std::int64_t kNoonJst = 1786064400;

astro::Observer tokyoObserver() {
  astro::Observer observer;
  observer.latitudeDegrees = kTokyoLatitude;
  observer.longitudeEastDegrees = kTokyoLongitudeEast;
  return observer;
}

app::Tick tickAt(std::uint32_t nowMillis) {
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

// 指定の局面に達するまで進める。到達できなければ false。
bool advanceTo(app::State& state, app::Phase wanted, std::uint32_t& clock,
               const app::Config& config) {
  for (int guard = 0; guard < 600; ++guard) {
    if (state.phase == wanted) {
      return true;
    }
    clock += 200;
    app::step(state, tickAt(clock), config, tokyoObserver());
  }
  return state.phase == wanted;
}

// スワイプを 1 回入れて、追尾に落ち着くまで進める。
void swipeForward(app::State& state, std::uint32_t& clock, const app::Config& config) {
  clock += 200;
  app::Tick swipe = tickAt(clock);
  swipe.input = app::Input::SwipeForward;
  app::step(state, swipe, config, tokyoObserver());
  advanceTo(state, app::Phase::Tracking, clock, config);
}

void testSwipeCyclesThroughAllTargets() {
  // スワイプ 8 回で一周し、すべての天体を通ること。
  // 実機のタッチセンサはこの Input を返すだけなので、ここが通れば
  // 実機の挙動もここで決まる。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  CHECK_TRUE(advanceTo(state, app::Phase::Tracking, clock, config));
  CHECK_TRUE(state.target == astro::Target::North);

  const astro::Target expected[] = {
      astro::Target::Sun,  astro::Target::Moon,    astro::Target::Mercury, astro::Target::Venus,
      astro::Target::Mars, astro::Target::Jupiter, astro::Target::Saturn,  astro::Target::North,
  };

  for (const astro::Target want : expected) {
    swipeForward(state, clock, config);
    CHECK_TRUE(state.target == want);
    // 切り替えるたびに解が更新されること
    CHECK_TRUE(state.hasSolve);
    CHECK_TRUE(state.lastPosition.valid);
  }
}

void testNeckPointsAtEachTarget() {
  // 本命。切り替えた天体に対し、首が実際にその方位を指すこと。
  //
  // 首の絶対方位 = 機体の向き + 首の相対角。これが天体の方位と一致すれば、
  // 実機で「首が太陽を指した」と言えるのと同じことを保証できる。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;

  // 機体は真北を向いている前提
  CHECK_TRUE(advanceTo(state, app::Phase::Tracking, clock, config));

  for (int step = 0; step < 8; ++step) {
    const astro::TargetPosition expected =
        astro::computeTargetPosition(state.target, kNoonJst, tokyoObserver(), true);
    CHECK_TRUE(expected.valid);

    const app::ServoIntent intent = app::servoIntentFor(state);
    const double neckAbsolute = static_cast<double>(intent.yawDeciDegrees) / 10.0;

    if (state.lastSolve.command.clampedYaw) {
      // 可動域を超える天体は端で止まる。指せていないことが残差に出ること。
      CHECK_TRUE(std::fabs(state.lastSolve.unreachableYawDegrees) > 0.0);
    } else {
      // 届く天体は、首の角度が天体の方位と一致すること
      CHECK_NEAR_ANGLE(neckAbsolute, expected.horizontal.azimuthDegrees, 0.2);
    }

    swipeForward(state, clock, config);
  }
}

void testBackwardSwipeReversesOrder() {
  // 逆スワイプで戻れること。実機で行き過ぎたときに戻せる必要がある。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  CHECK_TRUE(advanceTo(state, app::Phase::Tracking, clock, config));

  swipeForward(state, clock, config);
  CHECK_TRUE(state.target == astro::Target::Sun);

  clock += 200;
  app::Tick backward = tickAt(clock);
  backward.input = app::Input::SwipeBackward;
  app::step(state, backward, config, tokyoObserver());
  CHECK_TRUE(state.target == astro::Target::North);

  // 北から戻ると土星 (一周して末尾)
  clock += 200;
  app::Tick again = tickAt(clock);
  again.input = app::Input::SwipeBackward;
  app::step(state, again, config, tokyoObserver());
  CHECK_TRUE(state.target == astro::Target::Saturn);
}

void testTapTogglesAutoCycleAndItAdvances() {
  // タップで自動巡回が始まり、放っておいてもターゲットが進むこと。
  // 実機でスワイプが効かない場合の代替手段でもある。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  CHECK_TRUE(advanceTo(state, app::Phase::Tracking, clock, config));
  CHECK_TRUE(!state.autoCycleEnabled);

  clock += 200;
  app::Tick tap = tickAt(clock);
  tap.input = app::Input::Click;
  app::step(state, tap, config, tokyoObserver());
  CHECK_TRUE(state.autoCycleEnabled);

  // 巡回間隔を超えて進めると、ターゲットが自動で変わる
  const astro::Target before = state.target;
  for (int step = 0; step < 200; ++step) {
    clock += 200;
    app::step(state, tickAt(clock), config, tokyoObserver());
  }
  CHECK_TRUE(state.target != before);
  CHECK_TRUE(state.autoCycleEnabled);
}

void testEveryTargetProducesReachableCommand() {
  // どの天体を選んでも、サーボ指令が可動域に収まること。
  // ここが崩れると BSP 側で無言に clamp され、内部状態と実際がずれる。
  app::State state;
  app::Config config;
  std::uint32_t clock = 0;
  CHECK_TRUE(advanceTo(state, app::Phase::Tracking, clock, config));

  for (int step = 0; step < 8; ++step) {
    const app::ServoIntent intent = app::servoIntentFor(state);
    CHECK_TRUE(pointing::isYawReachable(intent.yawDeciDegrees));
    CHECK_TRUE(pointing::isPitchReachable(intent.pitchDeciDegrees));
    swipeForward(state, clock, config);
  }
}

} // namespace

int main() {
  testSwipeCyclesThroughAllTargets();
  testNeckPointsAtEachTarget();
  testBackwardSwipeReversesOrder();
  testTapTogglesAutoCycleAndItAdvances();
  testEveryTargetProducesReachableCommand();
  return testing::summarize("target-switch");
}
