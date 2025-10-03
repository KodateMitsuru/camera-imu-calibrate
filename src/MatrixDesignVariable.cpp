#include <kalibr_backend/MatrixDesignVariable.hpp>

namespace aslam {
namespace backend {

MatrixDesignVariable::MatrixDesignVariable(int rows, int cols)
    : DesignVariable(), matrix_(Eigen::MatrixXd::Identity(rows, cols)) {}

MatrixDesignVariable::MatrixDesignVariable(const Eigen::MatrixXd& matrix)
    : DesignVariable(), matrix_(matrix) {}

void MatrixDesignVariable::setMatrix(const Eigen::MatrixXd& matrix) {
  if (matrix.rows() != matrix_.rows() || matrix.cols() != matrix_.cols()) {
    throw std::runtime_error("MatrixDesignVariable: matrix size mismatch");
  }
  matrix_ = matrix;
}

void MatrixDesignVariable::updateImplementation(const double* dp, int size) {
  int expected_size = matrix_.rows() * matrix_.cols();
  if (size != expected_size) {
    throw std::runtime_error("MatrixDesignVariable: update size mismatch");
  }

  // Update matrix elements in column-major order (Eigen default)
  for (int i = 0; i < size; ++i) {
    int row = i % matrix_.rows();
    int col = i / matrix_.rows();
    matrix_(row, col) += dp[i];
  }
}

void MatrixDesignVariable::getParametersImplementation(
    Eigen::MatrixXd& value) const {
  int size = matrix_.rows() * matrix_.cols();
  value.resize(size, 1);

  // Store matrix elements in column-major order
  for (int col = 0; col < matrix_.cols(); ++col) {
    for (int row = 0; row < matrix_.rows(); ++row) {
      value(row + col * matrix_.rows(), 0) = matrix_(row, col);
    }
  }
}

void MatrixDesignVariable::setParametersImplementation(
    const Eigen::MatrixXd& value) {
  int expected_size = matrix_.rows() * matrix_.cols();
  if (value.rows() != expected_size || value.cols() != 1) {
    throw std::runtime_error("MatrixDesignVariable: parameters size mismatch");
  }

  // Restore matrix elements from column-major order
  for (int col = 0; col < matrix_.cols(); ++col) {
    for (int row = 0; row < matrix_.rows(); ++row) {
      matrix_(row, col) = value(row + col * matrix_.rows(), 0);
    }
  }
}

}  // namespace backend
}  // namespace aslam
