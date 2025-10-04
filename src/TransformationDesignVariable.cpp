#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <stdexcept>

namespace aslam {
namespace backend {

TransformationDesignVariable::TransformationDesignVariable()
    : initial_T_(sm::kinematics::Transformation()) {
  // Initialize with identity transformation
  Eigen::Vector4d q_identity(0.0, 0.0, 0.0, 1.0);  // w, x, y, z
  Eigen::Vector3d t_zero = Eigen::Vector3d::Zero();

  q_ = std::make_shared<RotationQuaternion>(q_identity);
  q_->setActive(true);

  t_ = std::make_shared<EuclideanPoint>(t_zero);
  t_->setActive(true);

  // Create the transformation expression
  expression_ =
      TransformationExpression(q_->toExpression(), t_->toExpression());
}

TransformationDesignVariable::TransformationDesignVariable(
    const sm::kinematics::Transformation& transformation, bool rotationActive,
    bool translationActive)
    : initial_T_(transformation) {
  // Extract quaternion (aslam uses [x, y, z, w] order)
  Eigen::Vector4d q_wxyz = transformation.q();
  Eigen::Vector4d q_xyzw;
  q_xyzw << q_wxyz[1], q_wxyz[2], q_wxyz[3],
      q_wxyz[0];  // Convert w,x,y,z to x,y,z,w

  q_ = std::make_shared<RotationQuaternion>(q_xyzw);
  q_->setActive(rotationActive);

  // Extract translation
  Eigen::Vector3d t = transformation.t();
  t_ = std::make_shared<EuclideanPoint>(t);
  t_->setActive(translationActive);

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

void TransformationDesignVariable::setRotationActive(bool active) {
  q_->setActive(active);
}

void TransformationDesignVariable::setTranslationActive(bool active) {
  t_->setActive(active);
}

}  // namespace backend
}  // namespace aslam
