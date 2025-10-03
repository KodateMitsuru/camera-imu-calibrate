#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <sm/kinematics/rotations.hpp>

namespace aslam {
namespace backend {

TransformationDesignVariable::TransformationDesignVariable()
    : DesignVariable(), T_() {}

TransformationDesignVariable::TransformationDesignVariable(
    const sm::kinematics::Transformation& T)
    : DesignVariable(), T_(T) {}

TransformationDesignVariable::TransformationDesignVariable(
    const Eigen::Matrix3d& C, const Eigen::Vector3d& t)
    : DesignVariable(), T_(C, t) {}

sm::kinematics::Transformation TransformationDesignVariable::transformation()
    const {
  return T_;
}

void TransformationDesignVariable::setTransformation(
    const sm::kinematics::Transformation& T) {
  T_ = T;
}

void TransformationDesignVariable::setParameters(const Eigen::Matrix3d& C,
                                                 const Eigen::Vector3d& t) {
  T_ = sm::kinematics::Transformation(C, t);
}

void TransformationDesignVariable::updateImplementation(const double* dp,
                                                        int size) {
  if (size != 6) {
    throw std::runtime_error(
        "TransformationDesignVariable: update size must be 6");
  }

  // First 3 elements are rotation perturbation (axis-angle)
  Eigen::Vector3d rotationPerturbation(dp[0], dp[1], dp[2]);

  // Last 3 elements are translation perturbation
  Eigen::Vector3d translationPerturbation(dp[3], dp[4], dp[5]);

  // Update rotation: C_new = exp(w_hat) * C_old
  Eigen::Matrix3d dC = sm::kinematics::axisAngle2R(rotationPerturbation);
  Eigen::Matrix3d C_new = dC * T_.C();

  // Update translation: t_new = t_old + dt
  Eigen::Vector3d t_new = T_.t() + translationPerturbation;

  // Set the new transformation
  T_ = sm::kinematics::Transformation(C_new, t_new);
}

void TransformationDesignVariable::getParametersImplementation(
    Eigen::MatrixXd& value) const {
  value.resize(6, 1);

  // Get rotation as axis-angle
  Eigen::Vector3d rotationVector = sm::kinematics::R2AxisAngle(T_.C());
  value.block<3, 1>(0, 0) = rotationVector;

  // Get translation
  value.block<3, 1>(3, 0) = T_.t();
}

void TransformationDesignVariable::setParametersImplementation(
    const Eigen::MatrixXd& value) {
  if (value.rows() != 6 || value.cols() != 1) {
    throw std::runtime_error(
        "TransformationDesignVariable: parameters must be 6x1");
  }

  // Extract rotation vector
  Eigen::Vector3d rotationVector = value.block<3, 1>(0, 0);
  Eigen::Matrix3d C = sm::kinematics::axisAngle2R(rotationVector);

  // Extract translation
  Eigen::Vector3d t = value.block<3, 1>(3, 0);

  // Set transformation
  T_ = sm::kinematics::Transformation(C, t);
}

}  // namespace backend
}  // namespace aslam
