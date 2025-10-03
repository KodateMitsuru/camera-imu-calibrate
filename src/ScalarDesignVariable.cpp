#include <kalibr_backend/ScalarDesignVariable.hpp>

namespace aslam {
namespace backend {

ScalarDesignVariable::ScalarDesignVariable(double value)
    : DesignVariable(), value_(value) {}

void ScalarDesignVariable::setValue(double value) { value_ = value; }

void ScalarDesignVariable::updateImplementation(const double* dp, int size) {
  if (size != 1) {
    throw std::runtime_error("ScalarDesignVariable: update size must be 1");
  }
  value_ += dp[0];
}

void ScalarDesignVariable::getParametersImplementation(
    Eigen::MatrixXd& value) const {
  value.resize(1, 1);
  value(0, 0) = value_;
}

void ScalarDesignVariable::setParametersImplementation(
    const Eigen::MatrixXd& value) {
  if (value.rows() != 1 || value.cols() != 1) {
    throw std::runtime_error("ScalarDesignVariable: parameters must be 1x1");
  }
  value_ = value(0, 0);
}

}  // namespace backend
}  // namespace aslam
