#include "compass/sphere_fit.hpp"

#include <cmath>

namespace compass {
namespace {

// 4x4 の連立方程式を部分ピボット選択つきガウス消去で解く。
// 解けなければ false。
bool solve4x4(double matrix[4][5], double solution[4]) {
  for (int column = 0; column < 4; ++column) {
    // ピボット選択。数値的に一番大きい行を持ってくる。
    int pivotRow = column;
    for (int row = column + 1; row < 4; ++row) {
      if (std::fabs(matrix[row][column]) > std::fabs(matrix[pivotRow][column])) {
        pivotRow = row;
      }
    }
    if (std::fabs(matrix[pivotRow][column]) < 1e-12) {
      return false;
    }
    if (pivotRow != column) {
      for (int index = column; index < 5; ++index) {
        const double swap = matrix[column][index];
        matrix[column][index] = matrix[pivotRow][index];
        matrix[pivotRow][index] = swap;
      }
    }

    for (int row = column + 1; row < 4; ++row) {
      const double factor = matrix[row][column] / matrix[column][column];
      for (int index = column; index < 5; ++index) {
        matrix[row][index] -= factor * matrix[column][index];
      }
    }
  }

  for (int row = 3; row >= 0; --row) {
    double value = matrix[row][4];
    for (int column = row + 1; column < 4; ++column) {
      value -= matrix[row][column] * solution[column];
    }
    solution[row] = value / matrix[row][row];
  }
  return true;
}

// NxN の連立方程式を部分ピボット選択つきガウス消去で解く。
// matrix は右辺を含む N x (N+1) の拡大係数行列。解けなければ false。
//
// Why not solve4x4 を一般化して置き換える: fitSphere は 4x4 固定で動いており、
// 触ると壊すリスクの方が大きい。楕円体側は 9x9 が要るので別に用意する。
template <int N> bool solveLinearSystem(double matrix[N][N + 1], double solution[N]) {
  for (int column = 0; column < N; ++column) {
    int pivotRow = column;
    for (int row = column + 1; row < N; ++row) {
      if (std::fabs(matrix[row][column]) > std::fabs(matrix[pivotRow][column])) {
        pivotRow = row;
      }
    }
    if (std::fabs(matrix[pivotRow][column]) < 1e-12) {
      return false;
    }
    if (pivotRow != column) {
      for (int index = column; index <= N; ++index) {
        const double swap = matrix[column][index];
        matrix[column][index] = matrix[pivotRow][index];
        matrix[pivotRow][index] = swap;
      }
    }

    for (int row = column + 1; row < N; ++row) {
      const double factor = matrix[row][column] / matrix[column][column];
      for (int index = column; index <= N; ++index) {
        matrix[row][index] -= factor * matrix[column][index];
      }
    }
  }

  for (int row = N - 1; row >= 0; --row) {
    double value = matrix[row][N];
    for (int column = row + 1; column < N; ++column) {
      value -= matrix[row][column] * solution[column];
    }
    solution[row] = value / matrix[row][row];
  }
  return true;
}

// 対称 3x3 の固有値分解を Jacobi 法で行う。
// input = eigenvectors * diag(eigenvalues) * eigenvectors^T となるように書き込む。
//
// Why not 特性方程式の解析解: 3 次方程式の閉じた形は固有値が近接するときに
// 桁落ちして固有ベクトルが直交しなくなる。Jacobi は各回転が直交変換なので
// 直交性が構造的に保たれ、補正行列を組むときに歪みを持ち込まない。
void jacobiEigenDecomposition(const double input[3][3], double eigenvalues[3],
                              double eigenvectors[3][3]) {
  double work[3][3];
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      work[row][column] = input[row][column];
      eigenvectors[row][column] = (row == column) ? 1.0 : 0.0;
    }
  }

  // 反復回数は固定。非対角成分が十分小さくなれば早期に打ち切る。
  constexpr int kMaxSweeps = 30;
  for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
    // 一番大きい非対角成分を消しにいく (古典 Jacobi)。
    int pivotRow = 0;
    int pivotColumn = 1;
    double largest = std::fabs(work[0][1]);
    if (std::fabs(work[0][2]) > largest) {
      largest = std::fabs(work[0][2]);
      pivotRow = 0;
      pivotColumn = 2;
    }
    if (std::fabs(work[1][2]) > largest) {
      largest = std::fabs(work[1][2]);
      pivotRow = 1;
      pivotColumn = 2;
    }
    const bool converged = largest < 1e-15;
    if (converged) {
      break;
    }

    // 該当成分をゼロにする回転角。tan(2t) = 2*a_pq / (a_qq - a_pp) を
    // オーバーフローに強い形で解く。
    const double diagonalDifference = work[pivotColumn][pivotColumn] - work[pivotRow][pivotRow];
    const double theta = diagonalDifference / (2.0 * work[pivotRow][pivotColumn]);
    const double sign = (theta >= 0.0) ? 1.0 : -1.0;
    const double tangent = sign / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
    const double cosine = 1.0 / std::sqrt(tangent * tangent + 1.0);
    const double sine = tangent * cosine;

    for (int index = 0; index < 3; ++index) {
      const double rowValue = work[index][pivotRow];
      const double columnValue = work[index][pivotColumn];
      work[index][pivotRow] = cosine * rowValue - sine * columnValue;
      work[index][pivotColumn] = sine * rowValue + cosine * columnValue;
    }
    for (int index = 0; index < 3; ++index) {
      const double rowValue = work[pivotRow][index];
      const double columnValue = work[pivotColumn][index];
      work[pivotRow][index] = cosine * rowValue - sine * columnValue;
      work[pivotColumn][index] = sine * rowValue + cosine * columnValue;
    }
    for (int index = 0; index < 3; ++index) {
      const double rowValue = eigenvectors[index][pivotRow];
      const double columnValue = eigenvectors[index][pivotColumn];
      eigenvectors[index][pivotRow] = cosine * rowValue - sine * columnValue;
      eigenvectors[index][pivotColumn] = sine * rowValue + cosine * columnValue;
    }
  }

  for (int index = 0; index < 3; ++index) {
    eigenvalues[index] = work[index][index];
  }
}

} // namespace

SphereFit fitSphere(const Vec3* points, std::size_t count) {
  SphereFit fit;
  if (points == nullptr || count < 4) {
    return fit;
  }

  // 正規方程式を組む。未知数は (cx, cy, cz, k) で k = r^2 - |c|^2。
  // 各サンプルは 2x*cx + 2y*cy + 2z*cz + k = x^2+y^2+z^2 という 1 式を与える。
  double matrix[4][5] = {};
  for (std::size_t index = 0; index < count; ++index) {
    const double x = points[index].x;
    const double y = points[index].y;
    const double z = points[index].z;
    const double squared = x * x + y * y + z * z;
    const double row[4] = {2.0 * x, 2.0 * y, 2.0 * z, 1.0};

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        matrix[i][j] += row[i] * row[j];
      }
      matrix[i][4] += row[i] * squared;
    }
  }

  double solution[4] = {};
  if (!solve4x4(matrix, solution)) {
    return fit;
  }

  const double centerX = solution[0];
  const double centerY = solution[1];
  const double centerZ = solution[2];
  const double radiusSquared =
      solution[3] + centerX * centerX + centerY * centerY + centerZ * centerZ;
  if (!(radiusSquared > 0.0)) {
    return fit;
  }

  fit.center.x = static_cast<float>(centerX);
  fit.center.y = static_cast<float>(centerY);
  fit.center.z = static_cast<float>(centerZ);
  fit.radius = static_cast<float>(std::sqrt(radiusSquared));

  // 残差。球面にどれだけ乗っているかで当てはめの質を見る。
  double residualSum = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    const double dx = points[index].x - centerX;
    const double dy = points[index].y - centerY;
    const double dz = points[index].z - centerZ;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double error = distance - fit.radius;
    residualSum += error * error;
  }
  const double rms = std::sqrt(residualSum / static_cast<double>(count));
  fit.normalizedResidual = static_cast<float>(rms / fit.radius);
  fit.valid = true;
  return fit;
}

float residualAfterCalibration(const MagCalibration& calibration, const Vec3* points,
                               std::size_t count) {
  if (!calibration.valid || points == nullptr || count == 0) {
    return 1.0F;
  }

  // 補正後の点の半径を集め、その平均からのばらつきを見る。
  double radiusSum = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    radiusSum += magnitude(applyCalibration(calibration, points[index]));
  }
  const double meanRadius = radiusSum / static_cast<double>(count);
  if (!(meanRadius > 0.0)) {
    return 1.0F;
  }

  double residualSum = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    const double error = magnitude(applyCalibration(calibration, points[index])) - meanRadius;
    residualSum += error * error;
  }
  const double rms = std::sqrt(residualSum / static_cast<double>(count));
  return static_cast<float>(rms / meanRadius);
}

EllipsoidFit fitEllipsoid(const Vec3* points, std::size_t count) {
  EllipsoidFit fit;
  // 未知数が 9 個あるので、最低でもそれを上回る点が要る。実際には姿勢の
  // 多様性も要るが、それは最後に残差で弾く。
  if (points == nullptr || count < 10) {
    return fit;
  }

  // 座標を代表スケールで割ってから設計行列を組む。
  //
  // Why not 生の値のまま解く: 実機の磁場は数百 uT あり、設計行列には 4 次の
  // 積 (x^2*x^2) が入るので成分が 10^10 規模まで広がる。9x9 の消去では
  // これが桁落ちして解が壊れる。無次元化すれば成分が 1 前後に揃う。
  double sumSquared = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    const double x = points[index].x;
    const double y = points[index].y;
    const double z = points[index].z;
    sumSquared += x * x + y * y + z * z;
  }
  const double scale = std::sqrt(sumSquared / static_cast<double>(count));
  if (!(scale > 0.0)) {
    return fit;
  }

  // 一般二次曲面 ax^2+by^2+cz^2+2dxy+2exz+2fyz+2gx+2hy+2iz = 1 の
  // 係数 9 個を最小二乗で求める。j = -1 に固定することで自明解を避けつつ
  // 線形最小二乗に落とせる (原点を通る曲面は磁場データには現れない)。
  double normal[9][10] = {};
  for (std::size_t index = 0; index < count; ++index) {
    const double x = points[index].x / scale;
    const double y = points[index].y / scale;
    const double z = points[index].z / scale;
    const double row[9] = {x * x,       y * y,   z * z,   2.0 * x * y, 2.0 * x * z,
                           2.0 * y * z, 2.0 * x, 2.0 * y, 2.0 * z};

    for (int i = 0; i < 9; ++i) {
      for (int j = 0; j < 9; ++j) {
        normal[i][j] += row[i] * row[j];
      }
      normal[i][9] += row[i];
    }
  }

  double coefficients[9] = {};
  if (!solveLinearSystem<9>(normal, coefficients)) {
    return fit;
  }

  // 二次形式の行列 A と一次項ベクトル。曲面は (p-c)^T A (p-c) = const の形。
  const double quadratic[3][3] = {
      {coefficients[0], coefficients[3], coefficients[4]},
      {coefficients[3], coefficients[1], coefficients[5]},
      {coefficients[4], coefficients[5], coefficients[2]},
  };
  double linear[3][4] = {
      {quadratic[0][0], quadratic[0][1], quadratic[0][2], -coefficients[6]},
      {quadratic[1][0], quadratic[1][1], quadratic[1][2], -coefficients[7]},
      {quadratic[2][0], quadratic[2][1], quadratic[2][2], -coefficients[8]},
  };

  // center = -A^-1 * (g,h,i)
  double center[3] = {};
  if (!solveLinearSystem<3>(linear, center)) {
    return fit;
  }

  // 中心へ平行移動したときの右辺。A*c = -L なので
  // (p-c)^T A (p-c) = 1 + c^T A c = 1 - c^T L となる。
  double constantTerm = 1.0;
  for (int index = 0; index < 3; ++index) {
    constantTerm -= center[index] * coefficients[6 + index];
  }
  // ゼロ近傍で割ると固有値が発散するので弾く。符号は問わない
  // (符号が反転していても、下の固有値が全部正かどうかで楕円体かを判定する)。
  if (std::fabs(constantTerm) < 1e-12) {
    return fit;
  }

  // A を定数で割って (p-c)^T M (p-c) = 1 に正規化する。M の固有値 λ に対し
  // その軸の半径は 1/sqrt(λ) になる。
  double normalized[3][3];
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      normalized[row][column] = quadratic[row][column] / constantTerm;
    }
  }

  double eigenvalues[3] = {};
  double eigenvectors[3][3] = {};
  jacobiEigenDecomposition(normalized, eigenvalues, eigenvectors);

  // 3 軸とも正の固有値でなければ楕円体ではない (双曲面など)。
  const bool isEllipsoid = eigenvalues[0] > 0.0 && eigenvalues[1] > 0.0 && eigenvalues[2] > 0.0;
  if (!isEllipsoid) {
    return fit;
  }

  // 各軸の半径と、揃える先の平均半径。
  double radii[3] = {};
  double radiusSum = 0.0;
  for (int index = 0; index < 3; ++index) {
    radii[index] = 1.0 / std::sqrt(eigenvalues[index]);
    radiusSum += radii[index];
  }
  const double meanRadius = radiusSum / 3.0;

  // 補正行列 = V * diag(meanRadius / radius) * V^T。
  // 固有軸方向に伸び縮みさせてから元の向きへ戻すので、傾いた楕円体も球になる。
  double correction[3][3] = {};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      double value = 0.0;
      for (int axis = 0; axis < 3; ++axis) {
        value += eigenvectors[row][axis] * (meanRadius / radii[axis]) * eigenvectors[column][axis];
      }
      correction[row][column] = value;
    }
  }

  // 無次元化を戻す。中心と半径は長さの次元を持つので scale を掛ける。
  // 補正行列は比なので無次元のままでよい。
  fit.center.x = static_cast<float>(center[0] * scale);
  fit.center.y = static_cast<float>(center[1] * scale);
  fit.center.z = static_cast<float>(center[2] * scale);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      fit.matrix[row][column] = static_cast<float>(correction[row][column]);
    }
  }
  fit.meanRadius = static_cast<float>(meanRadius * scale);
  fit.valid = true;

  // 残差は補正後の半径のばらつきを平均半径で割った値。
  double correctedRadiusSum = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    correctedRadiusSum += magnitude(applyEllipsoid(fit, points[index]));
  }
  const double correctedMean = correctedRadiusSum / static_cast<double>(count);
  if (!(correctedMean > 0.0)) {
    fit.valid = false;
    return fit;
  }

  double residualSum = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    const double error = magnitude(applyEllipsoid(fit, points[index])) - correctedMean;
    residualSum += error * error;
  }
  const double rms = std::sqrt(residualSum / static_cast<double>(count));
  fit.normalizedResidual = static_cast<float>(rms / correctedMean);
  return fit;
}

Vec3 applyEllipsoid(const EllipsoidFit& fit, Vec3 raw) {
  if (!fit.valid) {
    return raw;
  }

  const float centered[3] = {raw.x - fit.center.x, raw.y - fit.center.y, raw.z - fit.center.z};
  Vec3 corrected;
  corrected.x = fit.matrix[0][0] * centered[0] + fit.matrix[0][1] * centered[1] +
                fit.matrix[0][2] * centered[2];
  corrected.y = fit.matrix[1][0] * centered[0] + fit.matrix[1][1] * centered[1] +
                fit.matrix[1][2] * centered[2];
  corrected.z = fit.matrix[2][0] * centered[0] + fit.matrix[2][1] * centered[1] +
                fit.matrix[2][2] * centered[2];
  return corrected;
}

MagCalibration calibrationFromSphere(const SphereFit& fit, const Vec3* points, std::size_t count) {
  MagCalibration calibration;
  if (!fit.valid || points == nullptr || count == 0) {
    return calibration;
  }

  calibration.hardIronOffset = fit.center;

  // 中心を引いた後、各軸がどれだけ広がっているかを二乗平均で見る。
  // 等方な球なら 3 軸とも radius/sqrt(3) になる。
  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    const double dx = points[index].x - fit.center.x;
    const double dy = points[index].y - fit.center.y;
    const double dz = points[index].z - fit.center.z;
    sumX += dx * dx;
    sumY += dy * dy;
    sumZ += dz * dz;
  }

  const double rmsX = std::sqrt(sumX / static_cast<double>(count));
  const double rmsY = std::sqrt(sumY / static_cast<double>(count));
  const double rmsZ = std::sqrt(sumZ / static_cast<double>(count));
  const double mean = (rmsX + rmsY + rmsZ) / 3.0;
  if (!(mean > 0.0) || !(rmsX > 0.0) || !(rmsY > 0.0) || !(rmsZ > 0.0)) {
    return calibration;
  }

  calibration.softIronScale.x = static_cast<float>(mean / rmsX);
  calibration.softIronScale.y = static_cast<float>(mean / rmsY);
  calibration.softIronScale.z = static_cast<float>(mean / rmsZ);
  calibration.valid = true;
  return calibration;
}

} // namespace compass
