// compass_core が保証すること:
//  - キャリブレーションが既知のオフセット・スケールを復元し、条件不足では採用しない
//  - 傾斜補正が、傾けた機体でも元の方位を復元する (合成 → 復元の往復)
//  - 方位フィルタが円環量として正しく平均する (359 度と 1 度の平均が 0 度)
//  - 測定ゲートが、各棄却理由を意図した入力でだけ返す
//  - サーボバイアス表が学習した補正を返し、未学習ビンで暴れない
//  - 偏角の往復変換が可逆

#include "compass/calibration.hpp"
#include "compass/declination.hpp"
#include "compass/heading.hpp"
#include "compass/stability.hpp"

#include "test_support.hpp"

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846F;

compass::Vec3 rotateVector(compass::Vec3 value, float rollRadians, float pitchRadians) {
  // 機体を roll → pitch の順に傾けたとき、機体座標で観測される値を作る。
  // tiltCompensatedHeadingDegrees の逆変換にあたる。
  const float sinRoll = std::sin(rollRadians);
  const float cosRoll = std::cos(rollRadians);
  const float sinPitch = std::sin(pitchRadians);
  const float cosPitch = std::cos(pitchRadians);

  compass::Vec3 result;
  result.x = value.x * cosPitch - value.z * sinPitch;
  result.y = value.x * sinRoll * sinPitch + value.y * cosRoll + value.z * sinRoll * cosPitch;
  result.z = value.x * cosRoll * sinPitch - value.y * sinRoll + value.z * cosRoll * cosPitch;
  return result;
}

void testCalibrationRecovery() {
  // 既知のハードアイアン (10, -20, 5) と軸スケールを載せた合成球面から、
  // それらを復元できることを確認する。
  const compass::Vec3 trueOffset{10.0F, -20.0F, 5.0F};
  constexpr float kFieldRadius = 45.0F;

  compass::CalibrationCollector collector;
  for (int elevationStep = -8; elevationStep <= 8; ++elevationStep) {
    for (int azimuthStep = 0; azimuthStep < 24; ++azimuthStep) {
      const float elevation = (static_cast<float>(elevationStep) / 8.0F) * (kPi / 2.0F);
      const float azimuth = (static_cast<float>(azimuthStep) / 24.0F) * 2.0F * kPi;

      compass::Vec3 sample;
      sample.x = trueOffset.x + kFieldRadius * std::cos(elevation) * std::cos(azimuth);
      sample.y = trueOffset.y + kFieldRadius * std::cos(elevation) * std::sin(azimuth);
      sample.z = trueOffset.z + kFieldRadius * std::sin(elevation);
      collector.addSample(sample);
    }
  }

  const compass::MagCalibration calibration = collector.finish();
  CHECK_TRUE(calibration.valid);
  CHECK_NEAR(calibration.hardIronOffset.x, trueOffset.x, 0.5);
  CHECK_NEAR(calibration.hardIronOffset.y, trueOffset.y, 0.5);
  CHECK_NEAR(calibration.hardIronOffset.z, trueOffset.z, 0.5);
  // 等方な球面なのでスケールは 1 付近
  CHECK_NEAR(calibration.softIronScale.x, 1.0, 0.05);
  CHECK_NEAR(calibration.softIronScale.y, 1.0, 0.05);
  CHECK_NEAR(calibration.softIronScale.z, 1.0, 0.05);

  // 補正後は原点中心の球になる
  compass::Vec3 probe;
  probe.x = trueOffset.x + kFieldRadius;
  probe.y = trueOffset.y;
  probe.z = trueOffset.z;
  const compass::Vec3 corrected = compass::applyCalibration(calibration, probe);
  CHECK_NEAR(compass::magnitude(corrected), kFieldRadius, 1.5);
}

void testCalibrationRejectsInsufficientData() {
  // サンプル数が足りない
  compass::CalibrationCollector tooFewSamples;
  for (int index = 0; index < 10; ++index) {
    tooFewSamples.addSample(compass::Vec3{static_cast<float>(index), 0.0F, 0.0F});
  }
  CHECK_TRUE(!tooFewSamples.finish().valid);

  // サンプル数は足りるが、ある軸を回していない (レンジ不足)
  compass::CalibrationCollector flatRotation;
  for (int index = 0; index < 500; ++index) {
    const float azimuth = (static_cast<float>(index) / 500.0F) * 2.0F * kPi;
    flatRotation.addSample(
        compass::Vec3{45.0F * std::cos(azimuth), 45.0F * std::sin(azimuth), 0.0F});
  }
  CHECK_TRUE(!flatRotation.finish().valid);
  CHECK_TRUE(flatRotation.coverage() < 1.0F);

  // 未較正のキャリブレーションは入力を素通しする
  compass::MagCalibration unused;
  const compass::Vec3 raw{1.0F, 2.0F, 3.0F};
  const compass::Vec3 passthrough = compass::applyCalibration(unused, raw);
  CHECK_NEAR(passthrough.x, 1.0, 1e-6);
  CHECK_NEAR(passthrough.y, 2.0, 1e-6);
  CHECK_NEAR(passthrough.z, 3.0, 1e-6);
}

void testTiltCompensationRoundTrip() {
  // 本質的なテスト: 水平な機体で方位 H を作る磁場を用意し、機体を傾けたときに
  // 観測されるであろう値を合成して、傾斜補正が H を復元できるか見る。
  // 伏角 49 度 (日本付近) を入れて、水平成分だけでは復元できない状況を作る。
  constexpr float kInclination = 49.0F * kPi / 180.0F;
  constexpr float kFieldStrength = 46.0F;

  for (int headingStep = 0; headingStep < 12; ++headingStep) {
    const float headingDegrees = static_cast<float>(headingStep) * 30.0F;
    const float headingRadians = headingDegrees * kPi / 180.0F;

    // 水平姿勢での機体座標の磁場。x が機首方向。
    compass::Vec3 levelField;
    levelField.x = kFieldStrength * std::cos(kInclination) * std::cos(headingRadians);
    levelField.y = -kFieldStrength * std::cos(kInclination) * std::sin(headingRadians);
    levelField.z = kFieldStrength * std::sin(kInclination);

    // 水平姿勢では補正の有無にかかわらず方位が出る
    const compass::Attitude level;
    CHECK_NEAR_ANGLE(compass::tiltCompensatedHeadingDegrees(levelField, level), headingDegrees,
                     0.01);

    for (const float rollDegrees : {-25.0F, 0.0F, 25.0F}) {
      for (const float pitchDegrees : {-20.0F, 0.0F, 20.0F}) {
        const float roll = rollDegrees * kPi / 180.0F;
        const float pitch = pitchDegrees * kPi / 180.0F;

        const compass::Vec3 tiltedField = rotateVector(levelField, roll, pitch);
        compass::Attitude attitude;
        attitude.rollRadians = roll;
        attitude.pitchRadians = pitch;

        CHECK_NEAR_ANGLE(compass::tiltCompensatedHeadingDegrees(tiltedField, attitude),
                         headingDegrees, 0.05);
      }
    }
  }
}

void testAttitudeFromAccel() {
  // 静止・水平なら roll も pitch もゼロ (z に重力)
  const compass::Attitude level = compass::attitudeFromAccel(compass::Vec3{0.0F, 0.0F, 1.0F});
  CHECK_NEAR(level.rollRadians, 0.0, 1e-6);
  CHECK_NEAR(level.pitchRadians, 0.0, 1e-6);

  // 機首を上げると pitch が正
  const compass::Attitude noseUp = compass::attitudeFromAccel(compass::Vec3{-0.5F, 0.0F, 0.866F});
  CHECK_TRUE(noseUp.pitchRadians > 0.0F);

  // 右に傾けると roll が正
  const compass::Attitude rolled = compass::attitudeFromAccel(compass::Vec3{0.0F, 0.5F, 0.866F});
  CHECK_TRUE(rolled.rollRadians > 0.0F);
}

void testHeadingFilterIsCircular() {
  // 359 度と 1 度の平均は 0 度であって 180 度ではない。
  // 単純な移動平均だとここで破綻する。
  compass::HeadingFilter filter(0.5F);
  CHECK_TRUE(!filter.hasValue());

  filter.update(359.0F);
  filter.update(1.0F);
  CHECK_TRUE(filter.hasValue());
  CHECK_NEAR_ANGLE(filter.valueDegrees(), 0.0, 1.0);

  // 一定値を入れ続ければその値に収束する
  compass::HeadingFilter steady(0.5F);
  for (int index = 0; index < 40; ++index) {
    steady.update(123.4F);
  }
  CHECK_NEAR_ANGLE(steady.valueDegrees(), 123.4, 0.01);
  // 揃っていればばらつきは小さい
  CHECK_TRUE(steady.dispersionDegrees() < 1.0F);

  // ばらついた入力ではばらつきが大きく出る
  compass::HeadingFilter noisy(0.5F);
  for (int index = 0; index < 40; ++index) {
    noisy.update(index % 2 == 0 ? 0.0F : 90.0F);
  }
  CHECK_TRUE(noisy.dispersionDegrees() > 10.0F);

  filter.reset();
  CHECK_TRUE(!filter.hasValue());
}

void testMeasurementGate() {
  compass::MeasurementGateConfig config;
  config.settleMillis = 400;
  config.maxGyroDegPerSec = 3.0F;
  config.maxFieldDeviationRatio = 0.25F;
  config.maxHeadingDispersionDegrees = 3.0F;

  compass::MeasurementGate gate(config);
  gate.learnReferenceField(46.0F);
  CHECK_TRUE(gate.hasReferenceField());
  CHECK_NEAR(gate.referenceFieldMicroTesla(), 46.0, 1e-3);

  // 静穏な入力は通る
  compass::MeasurementGate::Input calm;
  calm.nowMillis = 10000;
  calm.servoMoving = false;
  calm.lastServoStopMillis = 9000;
  calm.gyroMagnitudeDegPerSec = 0.5F;
  calm.fieldMagnitudeMicroTesla = 46.0F;
  calm.headingDispersionDegrees = 0.5F;
  CHECK_TRUE(gate.accepts(calm));
  CHECK_TRUE(gate.evaluate(calm) == compass::MeasurementGate::Reject::None);

  // サーボが動いている
  compass::MeasurementGate::Input moving = calm;
  moving.servoMoving = true;
  CHECK_TRUE(gate.evaluate(moving) == compass::MeasurementGate::Reject::ServoMoving);

  // 停止直後 (settle 中)。境界: 399ms は棄却、400ms は通過。
  compass::MeasurementGate::Input settling = calm;
  settling.lastServoStopMillis = calm.nowMillis - 399;
  CHECK_TRUE(gate.evaluate(settling) == compass::MeasurementGate::Reject::Settling);
  settling.lastServoStopMillis = calm.nowMillis - 400;
  CHECK_TRUE(gate.evaluate(settling) == compass::MeasurementGate::Reject::None);

  // 機体自体が動いている
  compass::MeasurementGate::Input shaken = calm;
  shaken.gyroMagnitudeDegPerSec = 10.0F;
  CHECK_TRUE(gate.evaluate(shaken) == compass::MeasurementGate::Reject::DeviceMoving);

  // |B| が基準から外れている (サーボの保持電流を想定)
  compass::MeasurementGate::Input disturbed = calm;
  disturbed.fieldMagnitudeMicroTesla = 46.0F * 1.5F;
  CHECK_TRUE(gate.evaluate(disturbed) == compass::MeasurementGate::Reject::FieldAnomaly);

  // 方位が安定していない
  compass::MeasurementGate::Input unstable = calm;
  unstable.headingDispersionDegrees = 10.0F;
  CHECK_TRUE(gate.evaluate(unstable) == compass::MeasurementGate::Reject::Unstable);

  // 基準 |B| を学習していなければ、|B| 異常では弾かない
  compass::MeasurementGate withoutReference(config);
  CHECK_TRUE(!withoutReference.hasReferenceField());
  CHECK_TRUE(withoutReference.accepts(disturbed));

  // millis() の巻き戻り (32bit wrap) をまたいでも settle 判定が壊れない
  compass::MeasurementGate::Input wrapped = calm;
  wrapped.lastServoStopMillis = 0xFFFFFF00U;
  wrapped.nowMillis = 0xFFFFFF00U + 500U; // 巻き戻って 244
  CHECK_TRUE(gate.evaluate(wrapped) == compass::MeasurementGate::Reject::None);
}

void testMeasurementPoseGate() {
  // 実機では首の角度による誤差が 119 度に達し、他のどの要因より大きい。
  // 正面から外れた姿勢での測定は、他の条件が揃っていても弾くこと。
  compass::MeasurementGate gate;

  compass::MeasurementGate::Input calm;
  calm.nowMillis = 10000;
  calm.lastServoStopMillis = 9000;
  calm.gyroMagnitudeDegPerSec = 0.5F;
  calm.headingDispersionDegrees = 0.5F;
  calm.yawDeciDegrees = 0;
  CHECK_TRUE(gate.accepts(calm));

  // 首が振れていたら、静止していても採用しない
  compass::MeasurementGate::Input turned = calm;
  turned.yawDeciDegrees = 900;
  CHECK_TRUE(gate.evaluate(turned) == compass::MeasurementGate::Reject::NotMeasurementPose);

  // 姿勢の判定は他のどの棄却理由よりも優先される
  compass::MeasurementGate::Input turnedAndMoving = turned;
  turnedAndMoving.servoMoving = true;
  CHECK_TRUE(gate.evaluate(turnedAndMoving) ==
             compass::MeasurementGate::Reject::NotMeasurementPose);

  // 許容範囲の境界: ±2 度なら通り、それを超えたら弾く
  CHECK_TRUE(compass::isMeasurementPose(0));
  CHECK_TRUE(compass::isMeasurementPose(compass::kMeasurementYawToleranceDeci));
  CHECK_TRUE(compass::isMeasurementPose(-compass::kMeasurementYawToleranceDeci));
  CHECK_TRUE(!compass::isMeasurementPose(compass::kMeasurementYawToleranceDeci + 1));
  CHECK_TRUE(!compass::isMeasurementPose(-compass::kMeasurementYawToleranceDeci - 1));
}

void testServoBiasTable() {
  compass::ServoBiasTable table;
  CHECK_TRUE(!table.isPopulated());
  // 未学習なら補正しない
  CHECK_NEAR(table.correctionDegrees(0), 0.0, 1e-6);

  table.observe(0, 2.0F);
  CHECK_TRUE(table.isPopulated());
  CHECK_NEAR(table.correctionDegrees(0), 2.0, 1e-5);

  // 同じビンへの複数観測は平均される
  table.observe(0, 4.0F);
  CHECK_NEAR(table.correctionDegrees(0), 3.0, 1e-4);

  // 別のビンは独立
  table.observe(1200, -5.0F);
  CHECK_NEAR(table.correctionDegrees(1200), -5.0, 1e-5);
  CHECK_NEAR(table.correctionDegrees(0), 3.0, 1e-4);

  // 可動域外の入力でも添字が飛ばない
  CHECK_TRUE(compass::ServoBiasTable::binIndexFor(-99999) == 0);
  CHECK_TRUE(compass::ServoBiasTable::binIndexFor(99999) == compass::ServoBiasTable::kBinCount - 1);
  CHECK_TRUE(compass::ServoBiasTable::binIndexFor(-1280) == 0);
  CHECK_TRUE(compass::ServoBiasTable::binIndexFor(1280) == compass::ServoBiasTable::kBinCount - 1);

  // 誤差は円環量として平均される (359 度と 1 度の平均が 180 度にならない)
  compass::ServoBiasTable circular;
  circular.observe(0, 359.0F);
  circular.observe(0, 1.0F);
  const float averaged = circular.correctionDegrees(0);
  CHECK_TRUE(std::fabs(averaged) < 5.0F || std::fabs(averaged - 360.0F) < 5.0F);

  table.reset();
  CHECK_TRUE(!table.isPopulated());
  CHECK_TRUE(table.populatedBinCount() == 0);
}

void testDeclination() {
  // 日本は西偏 (東偏を負で表す)。磁北を向いているとき、真方位は小さくなる。
  CHECK_NEAR(compass::trueHeadingFromMagnetic(0.0F, -7.5F), 352.5, 1e-4);
  CHECK_NEAR(compass::trueHeadingFromMagnetic(10.0F, 5.0F), 15.0, 1e-4);

  // 往復で戻る
  for (const float heading : {0.0F, 45.0F, 179.0F, 359.9F}) {
    for (const float declination : {-9.0F, 0.0F, 12.0F}) {
      const float trueHeading = compass::trueHeadingFromMagnetic(heading, declination);
      CHECK_NEAR_ANGLE(compass::magneticHeadingFromTrue(trueHeading, declination), heading, 1e-3);
    }
  }

  // 常に [0,360)
  CHECK_TRUE(compass::trueHeadingFromMagnetic(1.0F, -10.0F) >= 0.0F);
  CHECK_TRUE(compass::trueHeadingFromMagnetic(1.0F, -10.0F) < 360.0F);
}

} // namespace

int main() {
  testCalibrationRecovery();
  testCalibrationRejectsInsufficientData();
  testTiltCompensationRoundTrip();
  testAttitudeFromAccel();
  testHeadingFilterIsCircular();
  testMeasurementGate();
  testMeasurementPoseGate();
  testServoBiasTable();
  testDeclination();
  return testing::summarize("compass");
}
