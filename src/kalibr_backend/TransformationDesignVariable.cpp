#include "format_utils.hpp"
#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <stdexcept>

namespace aslam {
namespace backend {

TransformationDesignVariable::TransformationDesignVariable(
    const sm::kinematics::Transformation& transformation, bool rotationActive,
    bool translationActive)
    : initial_T_(transformation) {
  // Extract quaternion (aslam uses [x, y, z, w] order)
  Eigen::Vector4d q = transformation.q();

  q_ = std::make_shared<RotationQuaternion>(q);
  q_->setActive(rotationActive);

  // Extract translation
  Eigen::Vector3d t = transformation.t();
  t_ = std::make_shared<EuclideanPoint>(t);
  t_->setActive(translationActive);
  
  basic_dv_ = std::make_shared<TransformationBasic>(
      q_->toExpression(), t_->toExpression());
  // Create the transformation expression from rotation and translation
  // expressions
  expression_ =
      TransformationExpression(q_->toExpression(), t_->toExpression());
}

TransformationExpression TransformationDesignVariable::toExpression() const {
  return expression_;
}

Eigen::Matrix4d TransformationDesignVariable::T() const {
  return expression_.toTransformationMatrix();
}

std::shared_ptr<DesignVariable> TransformationDesignVariable::getDesignVariable(
    int i) {
  if (i == 0) {
    return q_;
  } else if (i == 1) {
    return t_;
  } else {
    throw std::runtime_error("Index out of bounds: " + std::to_string(i) +
                             " >= 2");
  }
}


}  // namespace backend
}  // namespace aslam
