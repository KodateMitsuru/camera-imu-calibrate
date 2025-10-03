#ifndef KALIBR_BACKEND_MATRIX_DESIGN_VARIABLE_HPP
#define KALIBR_BACKEND_MATRIX_DESIGN_VARIABLE_HPP

#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>

namespace aslam {
namespace backend {

/**
 * @brief Design variable for a matrix (typically 3x3 for scale/misalignment)
 */
class MatrixDesignVariable : public DesignVariable {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor with identity matrix
   */
  explicit MatrixDesignVariable(int rows = 3, int cols = 3);

  /**
   * @brief Constructor with initial matrix
   */
  explicit MatrixDesignVariable(const Eigen::MatrixXd& matrix);

  virtual ~MatrixDesignVariable() = default;

  /**
   * @brief Get the matrix
   */
  Eigen::MatrixXd matrix() const { return matrix_; }

  /**
   * @brief Set the matrix
   */
  void setMatrix(const Eigen::MatrixXd& matrix);

  /**
   * @brief Get number of rows
   */
  int rows() const { return matrix_.rows(); }

  /**
   * @brief Get number of cols
   */
  int cols() const { return matrix_.cols(); }

 protected:
  /// \brief Update the design variable with a small perturbation
  virtual void updateImplementation(const double* dp, int size) override;

  /// \brief what is the number of dimensions of the perturbation variable
  virtual int minimalDimensionsImplementation() const override {
    return matrix_.rows() * matrix_.cols();
  }

  /// \brief Get the current parameters
  virtual void getParametersImplementation(
      Eigen::MatrixXd& value) const override;

  /// \brief Set the current parameters
  virtual void setParametersImplementation(
      const Eigen::MatrixXd& value) override;

 private:
  Eigen::MatrixXd matrix_;
};

}  // namespace backend
}  // namespace aslam

#endif  // KALIBR_BACKEND_MATRIX_DESIGN_VARIABLE_HPP
