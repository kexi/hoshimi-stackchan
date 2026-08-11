#pragma once

namespace pointing {

// StackChan-BSP の可動域。単位は 0.1 度 (deci-degree)。
// M5StackChan.cpp の servo_init() で定義されている実値。
//
// yaw:   -1280..1280 = -128.0..128.0 度。0 が正面。
// pitch:     0..900  =    0.0..90.0 度。0 が下端で負値を取れない。
inline constexpr int kYawMinDeci = -1280;
inline constexpr int kYawMaxDeci = 1280;
inline constexpr int kPitchMinDeci = 0;
inline constexpr int kPitchMaxDeci = 900;

// pitch の中央を「水平」に割り当てる。可動域の下端は原点ではないので、
// 高度 0 度は 0 ではなく 450 に対応する。
inline constexpr int kPitchLevelDeci = 450;

// pitch 1 度あたりの deci-degree。高度と 1:1 で対応させる。
//
// Why not 2:1 圧縮 (高度 -90..90 を pitch 0..900 に写す): 全高度を表現できるが、
// 「首が本当にその方向を指す」という要件を壊す。届かない範囲は clamp して
// UI で明示する方を選ぶ。日本の夏の南中太陽 (70-78 度) は上限を超える。
inline constexpr double kDeciDegreesPerAltitudeDegree = 10.0;

// この設計で表現できる高度の範囲 [度]
inline constexpr double kMaxRepresentableAltitude = 45.0;
inline constexpr double kMinRepresentableAltitude = -45.0;

// サーボの分解能。1 step = 0.3125 度 (= 3.125 deci-degree)。
// 天体計算の誤差 (最悪 0.1 度) より粗いので、精度の支配項はこちら。
inline constexpr double kServoStepDegrees = 0.3125;

struct ServoCommand {
  int yawDeciDegrees = 0;
  int pitchDeciDegrees = kPitchLevelDeci;
  bool clampedYaw = false;
  bool clampedPitch = false;
};

// 天体の高度 [度] → pitch の deci-degree。可動域外は端に貼りつく。
[[nodiscard]] int pitchDeciFromAltitude(double altitudeDegrees);

// pitch の deci-degree → 高度 [度]。
[[nodiscard]] double altitudeFromPitchDeci(int pitchDeci);

// 機体正面を基準にした相対方位 [度] → yaw の deci-degree。
// 入力は (-180, 180] に畳んでから使う。
[[nodiscard]] int yawDeciFromRelativeBearing(double relativeBearingDegrees);

// deci-degree の値が可動域に収まっているか。
[[nodiscard]] bool isYawReachable(int yawDeciDegrees);
[[nodiscard]] bool isPitchReachable(int pitchDeciDegrees);

// 実角度が指令角度へ両軸とも到達したか。
[[nodiscard]] bool isServoAtTarget(int actualYawDeciDegrees, int actualPitchDeciDegrees,
                                   int targetYawDeciDegrees, int targetPitchDeciDegrees,
                                   int toleranceDeciDegrees);

} // namespace pointing
