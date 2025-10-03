#ifndef KALIBR_BACKEND_SCALAR_DESIGN_VARIABLE_HPP
#define KALIBR_BACKEND_SCALAR_DESIGN_VARIABLE_HPP

#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>

namespace aslam {
namespace backend {

/**
 * @brief Design variable for a single scalar value
 */
class ScalarDesignVariable : public DesignVariable {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor with initial value
   */
  explicit ScalarDesignVariable(double value = 0.0);

  virtual ~ScalarDesignVariable() = default;

  /**
   * @brief Get the scalar value
   */
  double value() const { return value_; }

  /**
   * @brief Set the scalar value
   */
  void setValue(double value);

  /**
   * @brief Get value as expression
   */
  double toScalar() const { return value_; }

 protected:
  /// \brief Update the design variable with a small perturbation
  virtual void updateImplementation(const double* dp, int size) override;

  /// \brief what is the number of dimensions of the perturbation variable
  virtual int minimalDimensionsImplementation() const override { return 1; }

  /// \brief Get the current parameters
  virtual void getParametersImplementation(
      Eigen::MatrixXd& value) const override;

  /// \brief Set the current parameters
  virtual void setParametersImplementation(
      const Eigen::MatrixXd& value) override;

 private:
  double value_;
};

}  // namespace backend
}  // namespace aslam

#endif  // KALIBR_BACKEND_SCALAR_DESIGN_VARIABLE_HPP
