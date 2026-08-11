// ホストで動くシミュレータ。実機なしでロジックを確認するためのもの。
//
// 合成した磁気・加速度から方位を出し、指定の時刻・場所で天体を解いて、
// サーボ指令までを状態機械に通す。実機の挙動を再現するのではなく、
// 「計算が意図通りか」を目で見て確かめるのが目的。
//
//   simulator --list                 全ターゲットの方位・高度・サーボ指令を出す
//   simulator --target Venus         1 天体を詳しく
//   simulator --sweep 24             24 時間ぶんを 1 時間刻みで
//   simulator --states               状態機械を流して遷移を出す
//
// 時刻は --unix、場所は --lat/--lon、機体の向きは --heading で変えられる。

#include "app/state.hpp"
#include "astro/ephemeris.hpp"
#include "compass/declination.hpp"
#include "compass/heading.hpp"
#include "pointing/solver.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

struct Options {
  std::int64_t unixSeconds = 1786064400; // 2026-08-11 12:00 JST
  double latitude = 35.681236;
  double longitudeEast = 139.767125;
  float declinationEast = -7.9F;
  double bodyHeading = 0.0;
  std::string target;
  int sweepHours = 0;
  bool listAll = false;
  bool runStates = false;
};

astro::Observer observerFrom(const Options& options) {
  astro::Observer observer;
  observer.latitudeDegrees = options.latitude;
  observer.longitudeEastDegrees = options.longitudeEast;
  return observer;
}

const char* compassPoint(double azimuthDegrees) {
  static const char* kPoints[] = {"N",  "NNE", "NE", "ENE", "E",  "ESE", "SE", "SSE",
                                  "S",  "SSW", "SW", "WSW", "W",  "WNW", "NW", "NNW"};
  const int index = static_cast<int>(std::lround(azimuthDegrees / 22.5)) % 16;
  return kPoints[index < 0 ? index + 16 : index];
}

void printTargetRow(astro::Target target, const Options& options) {
  const astro::Observer observer = observerFrom(options);
  const astro::TargetPosition position =
      astro::computeTargetPosition(target, options.unixSeconds, observer, true);

  pointing::SolveInput input;
  input.targetAzimuthDegrees = position.horizontal.azimuthDegrees;
  input.targetAltitudeDegrees = position.horizontal.altitudeDegrees;
  input.bodyHeadingDegrees = options.bodyHeading;
  const pointing::SolveResult solved = pointing::solve(input);

  std::string flags;
  if (!position.aboveHorizon) {
    flags += " below-horizon";
  }
  if (solved.command.clampedYaw) {
    flags += " yaw-clamped";
  }
  if (solved.command.clampedPitch) {
    flags += " pitch-clamped";
  }
  if (flags.empty()) {
    flags = " ok";
  }

  std::printf("%-8s az=%7.2f (%-3s) alt=%+7.2f | yaw=%+5d pitch=%4d |%s\n",
              astro::targetName(target), position.horizontal.azimuthDegrees,
              compassPoint(position.horizontal.azimuthDegrees),
              position.horizontal.altitudeDegrees, solved.command.yawDeciDegrees,
              solved.command.pitchDeciDegrees, flags.c_str());
}

void printHeader(const Options& options) {
  std::printf("unix=%lld  lat=%.4f lon=%.4f  declination=%+.1f  bodyHeading=%.1f\n",
              static_cast<long long>(options.unixSeconds), options.latitude,
              options.longitudeEast, static_cast<double>(options.declinationEast),
              options.bodyHeading);
  std::printf("%-8s %-22s %-13s | %-16s |\n", "target", "azimuth", "altitude", "servo (deci-deg)");
  std::printf("--------------------------------------------------------------------------\n");
}

void commandList(const Options& options) {
  printHeader(options);
  for (std::uint8_t index = 0; index < static_cast<std::uint8_t>(astro::Target::kCount); ++index) {
    printTargetRow(static_cast<astro::Target>(index), options);
  }
}

bool parseTarget(const std::string& name, astro::Target& out) {
  for (std::uint8_t index = 0; index < static_cast<std::uint8_t>(astro::Target::kCount); ++index) {
    const auto candidate = static_cast<astro::Target>(index);
    if (name == astro::targetName(candidate)) {
      out = candidate;
      return true;
    }
  }
  return false;
}

void commandSweep(const Options& options) {
  astro::Target target = astro::Target::Sun;
  if (!options.target.empty() && !parseTarget(options.target, target)) {
    std::fprintf(stderr, "unknown target: %s\n", options.target.c_str());
    return;
  }

  std::printf("sweep %s over %d hours from unix=%lld\n", astro::targetName(target),
              options.sweepHours, static_cast<long long>(options.unixSeconds));
  std::printf("%-6s %-9s %-9s %-7s %s\n", "hour", "azimuth", "altitude", "yaw", "pitch");
  std::printf("------------------------------------------------\n");

  const astro::Observer observer = observerFrom(options);
  for (int hour = 0; hour < options.sweepHours; ++hour) {
    const std::int64_t when = options.unixSeconds + static_cast<std::int64_t>(hour) * 3600;
    const astro::TargetPosition position =
        astro::computeTargetPosition(target, when, observer, true);

    pointing::SolveInput input;
    input.targetAzimuthDegrees = position.horizontal.azimuthDegrees;
    input.targetAltitudeDegrees = position.horizontal.altitudeDegrees;
    input.bodyHeadingDegrees = options.bodyHeading;
    const pointing::SolveResult solved = pointing::solve(input);

    std::printf("%+5d  %8.2f  %+8.2f  %+6d  %5d%s\n", hour,
                position.horizontal.azimuthDegrees, position.horizontal.altitudeDegrees,
                solved.command.yawDeciDegrees, solved.command.pitchDeciDegrees,
                position.aboveHorizon ? "" : "  (below)");
  }
}

void commandStates(const Options& options) {
  // 状態機械を、測定が通る前提で流す。実機のタイミングを模した固定刻み。
  app::State state;
  app::Config config;
  const astro::Observer observer = observerFrom(options);

  std::printf("state machine trace (target=%s)\n", astro::targetName(state.target));
  std::printf("%-8s %-14s %-9s %s\n", "ms", "phase", "target", "servo");
  std::printf("------------------------------------------------------\n");

  app::Phase previousPhase = app::Phase::Error;
  for (std::uint32_t elapsed = 0; elapsed <= 200000; elapsed += 200) {
    app::Tick tick;
    tick.nowMillis = elapsed;
    tick.unixSeconds = options.unixSeconds + elapsed / 1000;
    tick.timeValid = true;
    tick.calibrationValid = true;
    tick.headingValid = true;
    tick.bodyTrueHeadingDegrees = static_cast<float>(options.bodyHeading);
    // Measuring に入ってから 600ms 経てば測定が通る、という想定
    tick.measurementAccepted = true;
    tick.servoSettled = true;

    // 30 秒でスワイプ、60 秒でクリック (自動巡回 ON) を入れてみる
    if (elapsed == 30000) {
      tick.input = app::Input::SwipeForward;
    } else if (elapsed == 60000) {
      tick.input = app::Input::Click;
    }

    app::step(state, tick, config, observer);

    const bool phaseChanged = state.phase != previousPhase;
    const bool isInput = elapsed == 30000 || elapsed == 60000;
    if (phaseChanged || isInput) {
      char servo[48] = "-";
      if (state.hasSolve) {
        std::snprintf(servo, sizeof(servo), "yaw=%+d pitch=%d",
                      state.lastSolve.command.yawDeciDegrees,
                      state.lastSolve.command.pitchDeciDegrees);
      }
      std::printf("%-8u %-14s %-9s %s%s\n", elapsed, app::phaseName(state.phase),
                  astro::targetName(state.target), servo,
                  state.autoCycleEnabled ? "  [auto]" : "");
      previousPhase = state.phase;
    }
  }
}

void commandTarget(const Options& options) {
  astro::Target target = astro::Target::Sun;
  if (!parseTarget(options.target, target)) {
    std::fprintf(stderr, "unknown target: %s\n", options.target.c_str());
    return;
  }

  const astro::Observer observer = observerFrom(options);
  const astro::TargetPosition position =
      astro::computeTargetPosition(target, options.unixSeconds, observer, true);

  pointing::SolveInput input;
  input.targetAzimuthDegrees = position.horizontal.azimuthDegrees;
  input.targetAltitudeDegrees = position.horizontal.altitudeDegrees;
  input.bodyHeadingDegrees = options.bodyHeading;
  const pointing::SolveResult solved = pointing::solve(input);

  std::printf("target        : %s\n", astro::targetName(target));
  std::printf("azimuth       : %.4f deg (%s)\n", position.horizontal.azimuthDegrees,
              compassPoint(position.horizontal.azimuthDegrees));
  std::printf("altitude      : %+.4f deg%s\n", position.horizontal.altitudeDegrees,
              position.aboveHorizon ? "" : "  (below horizon)");
  std::printf("body heading  : %.2f deg (true)\n", options.bodyHeading);
  std::printf("relative bear.: %+.4f deg\n",
              position.horizontal.azimuthDegrees - options.bodyHeading);
  std::printf("servo yaw     : %+d deci-deg (%+.2f deg)%s\n", solved.command.yawDeciDegrees,
              solved.command.yawDeciDegrees / 10.0,
              solved.command.clampedYaw ? "  CLAMPED" : "");
  std::printf("servo pitch   : %d deci-deg (%.2f deg)%s\n", solved.command.pitchDeciDegrees,
              solved.command.pitchDeciDegrees / 10.0,
              solved.command.clampedPitch ? "  CLAMPED" : "");
  if (solved.command.clampedYaw) {
    std::printf("  -> 首だけでは %.2f 度届かない。体ごと回す必要がある\n",
                solved.unreachableYawDegrees);
  }
  if (solved.command.clampedPitch) {
    std::printf("  -> 高度が %.2f 度ぶん可動域を超えている\n", solved.unreachablePitchDegrees);
  }
}

void printUsage() {
  std::printf("usage: simulator [options]\n"
              "  --list                 全ターゲットを一覧\n"
              "  --target <name>        1 天体を詳しく (Sun/Moon/Venus/...)\n"
              "  --sweep <hours>        指定時間ぶんを 1 時間刻みで\n"
              "  --states               状態機械の遷移を出す\n"
              "  --unix <seconds>       時刻 (既定: 2026-08-11 12:00 JST)\n"
              "  --lat <deg> --lon <deg>  観測地 (既定: 東京)\n"
              "  --declination <deg>    磁気偏角 (東偏が正、既定: -7.9)\n"
              "  --heading <deg>        機体正面の真方位 (既定: 0)\n");
}

} // namespace

int main(int argc, char** argv) {
  Options options;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const bool hasValue = index + 1 < argc;

    if (argument == "--list") {
      options.listAll = true;
    } else if (argument == "--states") {
      options.runStates = true;
    } else if (argument == "--target" && hasValue) {
      options.target = argv[++index];
    } else if (argument == "--sweep" && hasValue) {
      options.sweepHours = std::atoi(argv[++index]);
    } else if (argument == "--unix" && hasValue) {
      options.unixSeconds = std::atoll(argv[++index]);
    } else if (argument == "--lat" && hasValue) {
      options.latitude = std::atof(argv[++index]);
    } else if (argument == "--lon" && hasValue) {
      options.longitudeEast = std::atof(argv[++index]);
    } else if (argument == "--declination" && hasValue) {
      options.declinationEast = static_cast<float>(std::atof(argv[++index]));
    } else if (argument == "--heading" && hasValue) {
      options.bodyHeading = std::atof(argv[++index]);
    } else {
      printUsage();
      return argument == "--help" ? 0 : 1;
    }
  }

  if (options.runStates) {
    commandStates(options);
    return 0;
  }
  if (options.sweepHours > 0) {
    commandSweep(options);
    return 0;
  }
  if (!options.target.empty()) {
    commandTarget(options);
    return 0;
  }

  // 既定は一覧。引数なしで実行したときに何も出ないより有用。
  commandList(options);
  return 0;
}
