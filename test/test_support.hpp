#pragma once

// 依存を増やさずにホストテストを書くための最小の検査ヘルパ。
// 失敗は即座に報告し、最後に main が非ゼロで終了する。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace testing {

inline int& failureCount() {
  static int count = 0;
  return count;
}

inline void reportFailure(const char* file, int line, const std::string& message) {
  std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, message.c_str());
  ++failureCount();
}

inline bool checkTrue(bool value, const char* expression, const char* file, int line) {
  if (!value) {
    reportFailure(file, line, std::string("expected true: ") + expression);
  }
  return value;
}

inline bool checkNear(double actual, double expected, double tolerance, const char* expression,
                      const char* file, int line) {
  const double difference = std::fabs(actual - expected);
  if (!(difference <= tolerance)) {
    char buffer[512];
    std::snprintf(buffer, sizeof(buffer), "%s: actual=%.10g expected=%.10g diff=%.10g tol=%.10g",
                  expression, actual, expected, difference, tolerance);
    reportFailure(file, line, buffer);
    return false;
  }
  return true;
}

// 角度は 359.9 と 0.1 が近いので、単純な差ではなく最短角差で比べる。
inline bool checkNearAngle(double actualDegrees, double expectedDegrees, double toleranceDegrees,
                           const char* expression, const char* file, int line) {
  double difference = std::fmod(actualDegrees - expectedDegrees, 360.0);
  if (difference > 180.0) {
    difference -= 360.0;
  }
  if (difference < -180.0) {
    difference += 360.0;
  }
  if (!(std::fabs(difference) <= toleranceDegrees)) {
    char buffer[512];
    std::snprintf(buffer, sizeof(buffer), "%s: actual=%.10g expected=%.10g diff=%.10g tol=%.10g",
                  expression, actualDegrees, expectedDegrees, difference, toleranceDegrees);
    reportFailure(file, line, buffer);
    return false;
  }
  return true;
}

inline int summarize(const char* suiteName) {
  if (failureCount() == 0) {
    std::printf("PASS %s\n", suiteName);
    return 0;
  }
  std::fprintf(stderr, "FAILED %s: %d failure(s)\n", suiteName, failureCount());
  return 1;
}

} // namespace testing

#define CHECK_TRUE(value) ::testing::checkTrue((value), #value, __FILE__, __LINE__)
#define CHECK_NEAR(actual, expected, tolerance)                                                    \
  ::testing::checkNear((actual), (expected), (tolerance), #actual, __FILE__, __LINE__)
#define CHECK_NEAR_ANGLE(actual, expected, tolerance)                                              \
  ::testing::checkNearAngle((actual), (expected), (tolerance), #actual, __FILE__, __LINE__)
