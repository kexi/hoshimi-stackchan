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
  // 首を左右に振って、角度ごとの磁気のずれを覚える。
  // これが済むと、首を正面に戻さなくても方位が読めるようになり、
  // 持ち歩きながら天体を指し続けられる。
  LearningBias,
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

// 首のクセを学習するときに通す角度の数。可動域を等分に掃く。
// 16 ビンのうち半分以上を埋めたいので、それより多く採る。
inline constexpr std::uint8_t kLearningStepCount = 12;

// 学習の step 段目で首を向ける角度 [deci-degree]。
[[nodiscard]] int learningYawFor(std::uint8_t step);

// スワイプの向き。
enum class Input : std::uint8_t {
  None,
  SwipeForward,
  SwipeBackward,
  Click,
};

struct Config {
  // 方位を測り直す間隔。天体は動くが、方位は機体が動かない限り変わらない。
  // 定期的な測り直しの間隔。
  //
  // 機体が動いていないなら方位は変わらないので、本来は測り直す必要がない。
  // 20 秒にしていたときは、指した先から定期的に首が正面へ戻る動きが目立ち、
  // 「意味もなく首を振っている」ようにしか見えなかった。機体の動きは
  // ジャイロで拾えるので、これは保険として長めに置く。
  std::uint32_t remeasureIntervalMillis = 600000;
  // 自動巡回時に次のターゲットへ移る間隔。
  std::uint32_t autoCycleIntervalMillis = 8000;
  // サーボ指令を出してから収束したとみなすまでの時間。
  std::uint32_t servoSettleMillis = 1200;
  // 首を正面に戻してから磁気を測り始めるまでの待ち。
  // 必ず servoSettleMillis より長くすること。短いと首がまだ動いている最中に
  // Measuring へ進み、ゲートが servoMoving で弾き続けてタイムアウトし、
  // 汚れた方位がそのまま採用される。
  // 内訳: 首の移動 1200ms + 磁場の収束 300ms (実測) + 余裕。
  std::uint32_t measurePoseSettleMillis = 2000;
  // 測定を諦めるまでの時間。
  // 首が正面に落ち着いてからフィルタが溜まるまでを待てる長さにする。
  // 短いと、まだ空のフィルタを見て「測れなかった」と判断してしまう。
  std::uint32_t measureTimeoutMillis = 8000;
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

  // 首の角度によるずれを補正できるか (ServoBiasTable が学習済みか)。
  //
  // 真なら首を正面へ戻さずに測れるので、指したまま追尾を続けられる。
  // 持ち歩きながら観測するにはこれが要る。
  bool biasCorrected = false;
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

  // 直近に採用した機体の方位。Measuring で確定させ、機体が動くまで持ち続ける。
  //
  // Why not tick.headingValid をそのまま使う: 首を動かすと磁場が乱れて方位は
  // 測れなくなるが、機体は動いていないので方位自体は変わっていない。実機では
  // 指すために首を振った瞬間に headingValid が落ち、それを「方位を失った」と
  // 解釈して測定に戻り、また指しては戻るのを繰り返した。
  float bodyHeadingDegrees = 0.0F;
  bool hasHeading = false;

  // 首の角度によるずれを補正できるか。tick から写して持つ。
  // servoIntentFor() は State しか見ないので、ここに置く必要がある。
  bool biasCorrected = false;

  // 学習中に首を振っている段階。0 から kLearningStepCount-1 まで進む。
  std::uint8_t learningStep = 0;

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
