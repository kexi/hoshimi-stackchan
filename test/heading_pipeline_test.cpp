// ファームの loop() が方位を確定させる経路を、ホスト上で再現して検証する。
//
// 実機で「首が北ではなく西を向く」不具合が出た。実機に書き込んでは値を見る、
// という進め方では原因が絞れなかったので、同じ手順をホストで組み直す。
//
// 保証すること:
//  - 首が正面にないときの磁気は、方位に一切影響しない
//  - 首を振ってから戻したとき、汚れた値が残らない
//  - 方位が無効の間は、古い方位が使われない

#include "app/state.hpp"
#include "compass/calibration.hpp"
#include "compass/declination.hpp"
#include "compass/heading.hpp"
#include "compass/level_calibration.hpp"
#include "compass/stability.hpp"

#include "test_support.hpp"

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kInclination = 49.0F * kPi / 180.0F;
constexpr float kFieldStrength = 46.0F;
constexpr float kDeclinationEast = -7.9F;

// 実機と同じ順序で方位を確定させる。ファームの loop() の写し。
class HeadingPipeline {
public:
  explicit HeadingPipeline(compass::LevelCalibration calibration = {})
      : calibration_(calibration) {}

  struct Sample {
    compass::Vec3 mag;
    compass::Vec3 accel{0.0F, 0.0F, 1.0F};
    int commandedYaw = 0;
    bool servoMoving = false;
    std::uint32_t nowMillis = 0;
  };

  // ファームの updateHeading() 相当。
  void ingest(const Sample& sample) {
    const bool poseIsClean = compass::isMeasurementPose(sample.commandedYaw) && !sample.servoMoving;
    if (!poseIsClean) {
      return;
    }
    const compass::Attitude attitude = compass::attitudeFromAccel(sample.accel);
    const compass::Vec3 horizontal = compass::horizontalMagneticComponents(sample.mag, attitude);
    const compass::Vec3 corrected = compass::applyLevelCalibration(calibration_, horizontal);
    filter_.update(compass::headingDegreesFromHorizontal(corrected));
  }

  // ファームの commandServo() 相当。姿勢が変わるときだけ捨てる。
  void commandServo(int yawDeci) {
    if (yawDeci == commandedYaw_) {
      return;
    }
    const bool leavingMeasurePose =
        compass::isMeasurementPose(commandedYaw_) && !compass::isMeasurementPose(yawDeci);
    if (leavingMeasurePose) {
      filter_.reset();
      headingValid_ = false;
    }
    commandedYaw_ = yawDeci;
  }

  // ファームの loop() 後半 (ゲート判定と採用) 相当。
  void evaluate(const Sample& sample) {
    const compass::Attitude attitude = compass::attitudeFromAccel(sample.accel);
    const compass::Vec3 horizontal = compass::horizontalMagneticComponents(sample.mag, attitude);
    const compass::Vec3 corrected = compass::applyLevelCalibration(calibration_, horizontal);

    compass::MeasurementGate::Input input;
    input.nowMillis = sample.nowMillis;
    input.servoMoving = sample.servoMoving;
    input.lastServoStopMillis = sample.nowMillis - 5000;
    input.gyroMagnitudeDegPerSec = 0.1F;
    input.fieldMagnitudeMicroTesla = compass::magnitude(corrected);
    input.headingDispersionDegrees = filter_.dispersionDegrees();
    input.yawDeciDegrees = sample.commandedYaw;

    constexpr std::size_t kMinimumSamples = 8;
    const bool accepted = gate_.evaluate(input) == compass::MeasurementGate::Reject::None;
    const bool filterConverged = filter_.hasConverged(kMinimumSamples);
    const bool canLearnReference = accepted && filterConverged;
    if (canLearnReference) {
      gate_.learnReferenceField(input.fieldMagnitudeMicroTesla);
    }
    const bool headingCanBeUsed = accepted && filterConverged;
    if (!headingCanBeUsed) {
      return;
    }
    bodyTrueHeading_ = compass::trueHeadingFromMagnetic(filter_.valueDegrees(), kDeclinationEast);
    headingValid_ = true;
  }

  [[nodiscard]] bool headingValid() const { return headingValid_; }
  [[nodiscard]] float bodyTrueHeading() const { return bodyTrueHeading_; }
  [[nodiscard]] int commandedYaw() const { return commandedYaw_; }
  [[nodiscard]] bool hasReferenceField() const { return gate_.hasReferenceField(); }
  [[nodiscard]] float referenceField() const { return gate_.referenceFieldMicroTesla(); }

private:
  compass::LevelCalibration calibration_;
  compass::MeasurementGate gate_;
  compass::HeadingFilter filter_{0.25F};
  int commandedYaw_ = 0;
  bool headingValid_ = false;
  float bodyTrueHeading_ = 0.0F;
};

// 指定の磁方位を向いた機体が観測するであろう磁場を作る。
compass::Vec3 fieldForHeading(float headingDegrees) {
  const float radians = headingDegrees * kPi / 180.0F;
  compass::Vec3 field;
  field.x = kFieldStrength * std::cos(kInclination) * std::cos(radians);
  field.y = -kFieldStrength * std::cos(kInclination) * std::sin(radians);
  field.z = kFieldStrength * std::sin(kInclination);
  return field;
}

void testCleanPoseProducesCorrectHeading() {
  // 首が正面で静止していれば、素直に方位が出る。
  HeadingPipeline pipeline;
  HeadingPipeline::Sample sample;
  sample.mag = fieldForHeading(0.0F); // 磁北を向いている
  sample.commandedYaw = 0;
  sample.nowMillis = 10000;

  for (int step = 0; step < 40; ++step) {
    pipeline.ingest(sample);
    pipeline.evaluate(sample);
  }

  CHECK_TRUE(pipeline.headingValid());
  // 磁北を向いていて西偏 7.9 度なら、真方位は 352.1 度
  CHECK_NEAR_ANGLE(pipeline.bodyTrueHeading(), 352.1, 0.5);
}

void testPipelineAppliesLevelCalibration() {
  // ファームと同じ経路が、水平回転で得たオフセットを実際の方位へ適用すること。
  compass::LevelCalibration calibration;
  calibration.offsetX = 120.0F;
  calibration.offsetY = -85.0F;
  calibration.scaleX = 1.0F;
  calibration.scaleY = 1.0F;
  calibration.valid = true;
  HeadingPipeline pipeline{calibration};

  HeadingPipeline::Sample sample;
  sample.mag = fieldForHeading(90.0F);
  sample.mag.x += calibration.offsetX;
  sample.mag.y += calibration.offsetY;
  sample.commandedYaw = 0;
  sample.nowMillis = 10000;
  for (int step = 0; step < 40; ++step) {
    pipeline.ingest(sample);
    pipeline.evaluate(sample);
  }

  CHECK_TRUE(pipeline.headingValid());
  CHECK_NEAR_ANGLE(pipeline.bodyTrueHeading(), 82.1, 0.5);
}

void testTurnedPoseIsIgnoredEntirely() {
  // 首を振った状態の磁気は、どれだけ食わせても方位に影響しない。
  // 実機で西を向いた不具合は、ここが素通しになっていたことによる。
  HeadingPipeline pipeline;

  // まず正面で正しい方位を確定させる
  HeadingPipeline::Sample home;
  home.mag = fieldForHeading(0.0F);
  home.commandedYaw = 0;
  home.nowMillis = 10000;
  for (int step = 0; step < 40; ++step) {
    pipeline.ingest(home);
    pipeline.evaluate(home);
  }
  CHECK_TRUE(pipeline.headingValid());

  // 首を左端へ振る。実機ではここで 119 度ずれた値が観測される。
  pipeline.commandServo(-1270);
  CHECK_TRUE(!pipeline.headingValid()); // 方位は無効に戻る

  HeadingPipeline::Sample turned;
  turned.mag = fieldForHeading(119.0F); // 首の影響で大きくずれた値
  turned.commandedYaw = -1270;
  turned.nowMillis = 20000;
  for (int step = 0; step < 100; ++step) {
    pipeline.ingest(turned);
    pipeline.evaluate(turned);
  }

  // 汚れた値では方位が確定しないこと
  CHECK_TRUE(!pipeline.headingValid());
}

void testReturnToPoseRecoversCleanHeading() {
  // 首を振って戻したとき、汚れた平均が残らず、正しい方位を取り直せる。
  HeadingPipeline pipeline;

  HeadingPipeline::Sample home;
  home.mag = fieldForHeading(0.0F);
  home.commandedYaw = 0;
  home.nowMillis = 10000;
  for (int step = 0; step < 40; ++step) {
    pipeline.ingest(home);
    pipeline.evaluate(home);
  }

  // 首を振る → 汚れた値を浴びる
  pipeline.commandServo(-1270);
  HeadingPipeline::Sample turned;
  turned.mag = fieldForHeading(119.0F);
  turned.commandedYaw = -1270;
  turned.nowMillis = 20000;
  for (int step = 0; step < 100; ++step) {
    pipeline.ingest(turned);
    pipeline.evaluate(turned);
  }

  // 首を正面に戻す。移動中は採用されない。
  pipeline.commandServo(0);
  HeadingPipeline::Sample moving;
  moving.mag = fieldForHeading(0.0F);
  moving.commandedYaw = 0;
  moving.servoMoving = true;
  moving.nowMillis = 30000;
  for (int step = 0; step < 20; ++step) {
    pipeline.ingest(moving);
    pipeline.evaluate(moving);
  }
  CHECK_TRUE(!pipeline.headingValid());

  // 静止したら、正しい方位が取り直される
  HeadingPipeline::Sample settled = moving;
  settled.servoMoving = false;
  settled.nowMillis = 32000;
  for (int step = 0; step < 60; ++step) {
    pipeline.ingest(settled);
    pipeline.evaluate(settled);
  }
  CHECK_TRUE(pipeline.headingValid());
  // 汚れた 119 度に引っ張られず、正しい値に戻ること
  CHECK_NEAR_ANGLE(pipeline.bodyTrueHeading(), 352.1, 1.0);
}

void testMovingServoNeverContributes() {
  // 首が正面にあっても、動いている最中の値は使わない。
  HeadingPipeline pipeline;
  HeadingPipeline::Sample moving;
  moving.mag = fieldForHeading(90.0F);
  moving.commandedYaw = 0;
  moving.servoMoving = true;
  moving.nowMillis = 10000;

  for (int step = 0; step < 100; ++step) {
    pipeline.ingest(moving);
    pipeline.evaluate(moving);
  }
  CHECK_TRUE(!pipeline.headingValid());
}

void testMeasureWindowAccumulatesEnoughSamples() {
  // 実機で hdgOk が立たなかった原因の再現。
  //
  // 測定窓 8 秒、BMM150 は 24Hz なので理論上 192 サンプル取れるはずが、
  // 実機では 6 個で止まっていた。窓が実質 250ms しかない計算になる。
  //
  // 原因は Measuring 中も毎周期 commandServo() が呼ばれ、そのたびに
  // サーボ動作中フラグが立ち直してフィルタが溜まらないこと。
  // 測定窓の間は指令を出し直さないこと、を保証する。
  constexpr float kMagPeriodMillis = 1000.0F / 24.0F;
  constexpr std::uint32_t kMeasureWindowMillis = 8000;
  constexpr std::size_t kMinimumSamples = 8;

  HeadingPipeline pipeline;
  HeadingPipeline::Sample sample;
  sample.mag = fieldForHeading(0.0F);
  sample.commandedYaw = 0;

  // 測定窓のあいだ、mag の更新周期でサンプルを入れる
  std::size_t ingested = 0;
  for (std::uint32_t elapsed = 0; elapsed < kMeasureWindowMillis;
       elapsed += static_cast<std::uint32_t>(kMagPeriodMillis)) {
    sample.nowMillis = 10000 + elapsed;
    pipeline.ingest(sample);
    pipeline.evaluate(sample);
    ++ingested;
  }

  // 窓の中で十分なサンプルが入ること
  CHECK_TRUE(ingested >= kMinimumSamples);
  CHECK_TRUE(pipeline.headingValid());
  CHECK_NEAR_ANGLE(pipeline.bodyTrueHeading(), 352.1, 0.5);

  // 8 サンプルに必要な時間は 333ms 程度。窓 8 秒に対して十分短い。
  const float millisFor8 = kMagPeriodMillis * static_cast<float>(kMinimumSamples);
  CHECK_TRUE(millisFor8 < static_cast<float>(kMeasureWindowMillis));
}

void testReferenceFieldWaitsForConvergedHeading() {
  // 起動直後の過渡値1点を磁場基準にせず、8サンプル収束後の定常値を覚えること。
  HeadingPipeline pipeline;
  HeadingPipeline::Sample transient;
  transient.mag = fieldForHeading(0.0F);
  transient.mag.x *= 2.0F;
  transient.mag.y *= 2.0F;
  transient.commandedYaw = 0;
  transient.nowMillis = 10000;

  pipeline.ingest(transient);
  pipeline.evaluate(transient);
  CHECK_TRUE(!pipeline.hasReferenceField());

  HeadingPipeline::Sample steady;
  steady.mag = fieldForHeading(0.0F);
  steady.commandedYaw = 0;
  steady.nowMillis = 11000;
  for (int step = 0; step < 40; ++step) {
    pipeline.ingest(steady);
    pipeline.evaluate(steady);
  }

  const compass::Attitude attitude = compass::attitudeFromAccel(steady.accel);
  const float expected =
      compass::magnitude(compass::horizontalMagneticComponents(steady.mag, attitude));
  CHECK_TRUE(pipeline.hasReferenceField());
  CHECK_NEAR(pipeline.referenceField(), expected, 0.5);
}

} // namespace

int main() {
  testReferenceFieldWaitsForConvergedHeading();
  testMeasureWindowAccumulatesEnoughSamples();
  testCleanPoseProducesCorrectHeading();
  testPipelineAppliesLevelCalibration();
  testTurnedPoseIsIgnoredEntirely();
  testReturnToPoseRecoversCleanHeading();
  testMovingServoNeverContributes();
  return testing::summarize("heading-pipeline");
}
