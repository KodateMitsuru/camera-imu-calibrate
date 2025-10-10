#ifndef KALIBR_BACKEND_TRANSFORMATION_DESIGN_VARIABLE_HPP
#define KALIBR_BACKEND_TRANSFORMATION_DESIGN_VARIABLE_HPP

#include "aslam/backend/TransformationBasic.hpp"
#include <Eigen/Core>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <memory>
#include <sm/kinematics/Transformation.hpp>

namespace aslam {
namespace backend {

/**
 * @brief Design variable for SE(3) transformation (rotation + translation)
 * This is a composite design variable consisting of:
 * - RotationQuaternion for rotation
 * - EuclideanPoint for translation
 */
class TransformationDesignVariable {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor with initial transformation
   * @param transformation Initial SE(3) transformation
   * @param rotationActive Whether rotation is active for optimization
   * @param translationActive Whether translation is active for optimization
   */
  TransformationDesignVariable(
      const sm::kinematics::Transformation& transformation,
      bool rotationActive = true, bool translationActive = true);

  virtual ~TransformationDesignVariable() = default;

  /**
   * @brief Convert to TransformationExpression for optimization
   */
  TransformationExpression toExpression() const;

  /**
   * @brief Get the current transformation matrix
   */
  Eigen::Matrix4d T() const;

  /**
   * @brief Get the number of design variables (always 2: rotation +
   * translation)
   */
  int numDesignVariables() const { return 2; }

  /**
   * @brief Get design variable by index
   * @param i Index (0 = rotation, 1 = translation)
   */
  std::shared_ptr<DesignVariable> getDesignVariable(int i);

  std::shared_ptr<DesignVariable> designVariable(int i) {
    return getDesignVariable(i);
  }

 private:
  sm::kinematics::Transformation initial_T_;
  std::shared_ptr<RotationQuaternion> q_;  // Rotation quaternion
  std::shared_ptr<EuclideanPoint> t_;      // Translation vector
  std::shared_ptr<TransformationBasic> basic_dv_;  // Combined expression
  TransformationExpression expression_;    // Combined expression
};

}  // namespace backend
}  // namespace aslam

#endif  // KALIBR_BACKEND_TRANSFORMATION_DESIGN_VARIABLE_HPP
