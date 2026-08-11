#include "compass/level_calibration.hpp"

#include <array>
#include <cmath>

namespace compass {
namespace {

constexpr float kPi = 3.14159265358979323846F;
// 円周を何分割して被覆を測るか。一回転すれば全ビンが埋まる。
constexpr std::size_t kCoverageBins = 24;
// 中央値を求めるのに使う標本数の上限。全点を並べ替える必要はない。
constexpr std::size_t kMaxSamplesForMedian = 64;

// 3x3 の連立方程式をガウス消去で解く。円の当てはめ用。
bool solve3x3(double matrix[3][4], double solution[3]) {
  for (int column = 0; column < 3; ++column) {
    int pivotRow = column;
    for (int row = column + 1; row < 3; ++row) {
      if (std::fabs(matrix[row][column]) > std::fabs(matrix[pivotRow][column])) {
        pivotRow = row;
      }
    }
    if (std::fabs(matrix[pivotRow][column]) < 1e-12) {
      return false;
    }
    if (pivotRow != column) {
      for (int index = column; index < 4; ++index) {
        const double swap = matrix[column][index];
        matrix[column][index] = matrix[pivotRow][index];
        matrix[pivotRow][index] = swap;
      }
    }
    for (int row = column + 1; row < 3; ++row) {
      const double factor = matrix[row][column] / matrix[column][column];
      for (int index = column; index < 4; ++index) {
        matrix[row][index] -= factor * matrix[column][index];
      }
    }
  }
  for (int row = 2; row >= 0; --row) {
    double value = matrix[row][3];
    for (int column = row + 1; column < 3; ++column) {
      value -= matrix[row][column] * solution[column];
    }
    solution[row] = value / matrix[row][row];
  }
  return true;
}

} // namespace

float angularCoverage(const Vec3* points, std::size_t count, float centerX, float centerY) {
  if (points == nullptr || count == 0) {
    return 0.0F;
  }

  std::array<bool, kCoverageBins> visited{};
  for (std::size_t index = 0; index < count; ++index) {
    const float dx = points[index].x - centerX;
    const float dy = points[index].y - centerY;
    // 中心のごく近くの点は角度が定まらないので数えない。
    if (std::hypot(dx, dy) < 1.0F) {
      continue;
    }
    float angle = std::atan2(dy, dx);
    if (angle < 0.0F) {
      angle += 2.0F * kPi;
    }
    const auto bin =
        static_cast<std::size_t>(angle / (2.0F * kPi) * static_cast<float>(kCoverageBins));
    visited[bin < kCoverageBins ? bin : kCoverageBins - 1] = true;
  }

  std::size_t filled = 0;
  for (const bool seen : visited) {
    if (seen) {
      ++filled;
    }
  }
  return static_cast<float>(filled) / static_cast<float>(kCoverageBins);
}

LevelCalibration fitLevelCircle(const Vec3* points, std::size_t count) {
  LevelCalibration calibration;
  if (points == nullptr || count < 8) {
    return calibration;
  }

  // 円 |p - c|^2 = r^2 を展開すると 2*cx*x + 2*cy*y + k = x^2 + y^2 で線形。
  // 未知数は (cx, cy, k)、k = r^2 - |c|^2。
  double matrix[3][4] = {};
  for (std::size_t index = 0; index < count; ++index) {
    const double x = points[index].x;
    const double y = points[index].y;
    const double squared = x * x + y * y;
    const double row[3] = {2.0 * x, 2.0 * y, 1.0};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        matrix[i][j] += row[i] * row[j];
      }
      matrix[i][3] += row[i] * squared;
    }
  }

  double solution[3] = {};
  if (!solve3x3(matrix, solution)) {
    return calibration;
  }

  const double centerX = solution[0];
  const double centerY = solution[1];
  const double radiusSquared = solution[2] + centerX * centerX + centerY * centerY;
  if (!(radiusSquared > 0.0)) {
    return calibration;
  }
  const double radius = std::sqrt(radiusSquared);

  // 円周をどれだけ覆えたか。半周しか回していないと中心がずれるので、
  // ここを見ないと不完全な回転を採用してしまう。
  const float coverage =
      angularCoverage(points, count, static_cast<float>(centerX), static_cast<float>(centerY));
  if (coverage < 0.75F) {
    return calibration;
  }

  // 中央値から大きく外れた点を除く。
  //
  // 回転中に姿勢が揺れると、その点だけ別の断面を見ることになり半径が変わる。
  // 実機では半径が 2〜22uT に散らばり、そのまま使うと方位誤差 23 度になった。
  // 外れ値を落とすと 11 度まで下がる。
  double radii[kMaxSamplesForMedian] = {};
  const std::size_t sampled = count < kMaxSamplesForMedian ? count : kMaxSamplesForMedian;
  const std::size_t stride = count / sampled;
  for (std::size_t index = 0; index < sampled; ++index) {
    const Vec3& point = points[index * stride];
    radii[index] = std::hypot(point.x - centerX, point.y - centerY);
  }
  // 挿入ソート。要素数が小さいので十分速い。
  for (std::size_t i = 1; i < sampled; ++i) {
    const double key = radii[i];
    std::size_t j = i;
    while (j > 0 && radii[j - 1] > key) {
      radii[j] = radii[j - 1];
      --j;
    }
    radii[j] = key;
  }
  const double medianRadius = radii[sampled / 2];
  const double outlierBand = 0.4 * medianRadius;

  // X と Y の感度差を測る。円が楕円になっているぶんを揃える。
  double sumX = 0.0;
  double sumY = 0.0;
  std::size_t inlierCount = 0;
  for (std::size_t index = 0; index < count; ++index) {
    const double dx = points[index].x - centerX;
    const double dy = points[index].y - centerY;
    if (std::fabs(std::hypot(dx, dy) - medianRadius) > outlierBand) {
      continue;
    }
    sumX += dx * dx;
    sumY += dy * dy;
    ++inlierCount;
  }
  if (inlierCount < 8) {
    return calibration;
  }
  const double rmsX = std::sqrt(sumX / static_cast<double>(inlierCount));
  const double rmsY = std::sqrt(sumY / static_cast<double>(inlierCount));
  if (!(rmsX > 0.0) || !(rmsY > 0.0)) {
    return calibration;
  }
  const double meanRms = 0.5 * (rmsX + rmsY);

  calibration.offsetX = static_cast<float>(centerX);
  calibration.offsetY = static_cast<float>(centerY);
  calibration.scaleX = static_cast<float>(meanRms / rmsX);
  calibration.scaleY = static_cast<float>(meanRms / rmsY);
  calibration.radius = static_cast<float>(radius);

  // 補正後の半径のばらつきで質を測る。外れ値は採否の判断からも除く。
  // 姿勢が揺れた点まで数えると、良いキャリブレーションでも残差が大きく出る。
  double residualSum = 0.0;
  double correctedRadiusSum = 0.0;
  std::size_t scored = 0;
  for (std::size_t index = 0; index < count; ++index) {
    const double rawDx = points[index].x - centerX;
    const double rawDy = points[index].y - centerY;
    if (std::fabs(std::hypot(rawDx, rawDy) - medianRadius) > outlierBand) {
      continue;
    }
    correctedRadiusSum += std::hypot(rawDx * calibration.scaleX, rawDy * calibration.scaleY);
    ++scored;
  }
  if (scored == 0) {
    return calibration;
  }
  const double meanCorrected = correctedRadiusSum / static_cast<double>(scored);
  if (!(meanCorrected > 0.0)) {
    return calibration;
  }
  for (std::size_t index = 0; index < count; ++index) {
    const double rawDx = points[index].x - centerX;
    const double rawDy = points[index].y - centerY;
    if (std::fabs(std::hypot(rawDx, rawDy) - medianRadius) > outlierBand) {
      continue;
    }
    const double error =
        std::hypot(rawDx * calibration.scaleX, rawDy * calibration.scaleY) - meanCorrected;
    residualSum += error * error;
  }
  calibration.normalizedResidual =
      static_cast<float>(std::sqrt(residualSum / static_cast<double>(scored)) / meanCorrected);
  // 半径も外れ値を除いた値にする。UI と採否の閾値がこれを見る。
  calibration.radius = static_cast<float>(meanCorrected);
  calibration.valid = true;
  return calibration;
}

Vec3 applyLevelCalibration(const LevelCalibration& calibration, Vec3 raw) {
  if (!calibration.valid) {
    return raw;
  }
  Vec3 result;
  result.x = (raw.x - calibration.offsetX) * calibration.scaleX;
  result.y = (raw.y - calibration.offsetY) * calibration.scaleY;
  // Z は方位計算に使わないので素通し。水平回転では Z のオフセットが決まらない。
  result.z = raw.z;
  return result;
}

} // namespace compass
