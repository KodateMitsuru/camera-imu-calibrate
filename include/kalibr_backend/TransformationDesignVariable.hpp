#ifndef KALIBR_BACKEND_TRANSFORMATION_DESIGN_VARIABLE_HPP
#define KALIBR_BACKEND_TRANSFORMATION_DESIGN_VARIABLE_HPP

#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/Transformation.hpp>

namespace aslam {
namespace backend {

/**
 * @brief Design variable for SE(3) transformation (rotation + translation)
 */
class TransformationDesignVariable : public DesignVariable {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor with identity transformation
   */
  TransformationDesignVariable();

  /**
   * @brief Constructor with initial transformation
   */
  TransformationDesignVariable(const sm::kinematics::Transformation& T);

  /**
   * @brief Constructor with rotation and translation
   */
  TransformationDesignVariable(const Eigen::Matrix3d& C,
                               const Eigen::Vector3d& t);

  virtual ~TransformationDesignVariable() = default;

  /**
   * @brief Get the transformation
   */
  sm::kinematics::Transformation transformation() const;

  /**
   * @brief Get rotation matrix
   */
  Eigen::Matrix3d C() const { return T_.C(); }

  /**
   * @brief Get translation vector
   */
  Eigen::Vector3d t() const { return T_.t(); }

  /**
   * @brief Get 4x4 transformation matrix
   */
  Eigen::Matrix4d T() const { return T_.T(); }

  /**
   * @brief Set the transformation
   */
  void setTransformation(const sm::kinematics::Transformation& T);

  /**
   * @brief Set from rotation and translation
   */
  void setParameters(const Eigen::Matrix3d& C, const Eigen::Vector3d& t);

 protected:
  /// \brief Update the design variable with a small perturbation
  virtual void updateImplementation(const double* dp, int size) override;

  /// \brief what is the number of dimensions of the perturbation variable
  virtual int minimalDimensionsImplementation() const override { return 6; }

  /// \brief Get the current parameters (rotation vector + translation)
  virtual void getParametersImplementation(
      Eigen::MatrixXd& value) const override;

  /// \brief Set the current parameters
  virtual void setParametersImplementation(
      const Eigen::MatrixXd& value) override;

 private:
  sm::kinematics::Transformation T_;
};

}  // namespace backend
}  // namespace aslam

#endif  // KALIBR_BACKEND_TRANSFORMATION_DESIGN_VARIABLE_HPP
