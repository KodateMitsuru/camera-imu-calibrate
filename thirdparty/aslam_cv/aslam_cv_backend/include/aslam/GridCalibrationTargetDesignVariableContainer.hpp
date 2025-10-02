#ifndef ASLAM_GRID_CALIBRATION_TARGET_DESIGN_VARIABLE_CONTAINER_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_DESIGN_VARIABLE_CONTAINER_HPP

#include <aslam/backend/MappedEuclideanPoint.hpp>
#include <aslam/targets.hpp>
#include <memory>

namespace aslam {

class GridCalibrationTargetDesignVariableContainer {
 public:
  GridCalibrationTargetDesignVariableContainer(
      std::shared_ptr<cameras::GridCalibrationTargetBase> target, bool active);
  virtual ~GridCalibrationTargetDesignVariableContainer();

  /// \brief get all underlying design variables.
  void getDesignVariables(
      backend::DesignVariable::set_t& designVariables) const;

  /// \brief estimate the ith point.
  void setPointActive(size_t i, bool active);

  /// \brief is point i being estimated?
  bool isPointActive(size_t i);

  /// \brief get the expression for point i
  backend::EuclideanExpression getPoint(size_t i);

  /// \brief get the target.
  std::shared_ptr<cameras::GridCalibrationTargetBase> getTarget();

 private:
  std::shared_ptr<cameras::GridCalibrationTargetBase> _target;

  std::vector<std::shared_ptr<backend::MappedEuclideanPoint> > _points;
  std::vector<backend::EuclideanExpression> _pointExpressions;
};

}  // namespace aslam

#endif /* ASLAM_GRID_CALIBRATION_TARGET_DESIGN_VARIABLE_CONTAINER_HPP */
