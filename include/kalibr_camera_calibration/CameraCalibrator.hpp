#ifndef KALIBR_CAMERA_CALIBRATOR_HPP
#define KALIBR_CAMERA_CALIBRATOR_HPP

#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/OptimizationProblem.h>

#include <Eigen/Core>
#include <aslam/Time.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <format_utils.hpp>
#include <functional>
#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <kalibr_common/ConfigReader.hpp>
#include <kalibr_common/ImageDatasetReader.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "sm/kinematics/Transformation.hpp"

namespace kalibr {

// Design variable group IDs
inline constexpr int CAM_CALIBRATION_GROUP_ID = 0;
inline constexpr int TRANSFORMATION_GROUP_ID = 1;
inline constexpr int LANDMARK_GROUP_ID = 2;

/**
 * @brief Exception thrown when optimization diverges
 */
class OptimizationDiverged : public std::runtime_error {
 public:
  OptimizationDiverged()
      : std::runtime_error("Optimization did not converge in maxIterations") {}
  explicit OptimizationDiverged(const std::string& msg)
      : std::runtime_error(msg) {}
};

/**
 * @brief Target detector wrapper
 *
 * Wraps the calibration target and grid detector
 */
class TargetDetector {
 public:
  /**
   * @brief Construct target detector
   * @param targetConfig Target configuration parameters
   * @param cameraGeometry Camera geometry for detection
   * @param showCorners Show corner detection visualization
   * @param showReproj Show reprojection visualization
   * @param showOneStep Single-step image processing
   */
  TargetDetector(
      const CalibrationTargetParameters& targetConfig,
      const std::shared_ptr<aslam::cameras::CameraGeometryBase>& cameraGeometry,
      bool showCorners = false, bool showReproj = false,
      bool showOneStep = false);

  // Accessors
  std::shared_ptr<aslam::cameras::GridCalibrationTargetBase> getGrid() const {
    return grid_;
  }
  std::shared_ptr<aslam::cameras::GridDetector> getDetector() const {
    return detector_;
  }
  const CalibrationTargetParameters& getTargetConfig() const {
    return targetConfig_;
  }

 private:
  CalibrationTargetParameters targetConfig_;
  std::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid_;
  std::shared_ptr<aslam::cameras::GridDetector> detector_;
};

/**
 * @brief Camera geometry wrapper for calibration
 *
 * Wraps camera model with design variables for optimization
 */
class CameraGeometry {
 public:
  /**
   * @brief Construct camera geometry
   * @param cameraModel Camera parameters
   * @param targetConfig Target configuration
   * @param dataset Image dataset
   * @param geometry Pre-existing camera geometry (optional)
   * @param verbose Enable verbose output
   */
  CameraGeometry(
      const CameraParameters& cameraModel,
      const CalibrationTargetParameters& targetConfig,
      const ImageDatasetReader& dataset,
      std::shared_ptr<aslam::cameras::CameraGeometryBase> geometry = nullptr,
      bool verbose = false);

  /**
   * @brief Set design variable active status
   * @param projectionActive Projection parameters active
   * @param distortionActive Distortion parameters active
   * @param shutterActive Shutter parameters active
   */
  void setDvActiveStatus(bool projectionActive, bool distortionActive,
                         bool shutterActive);

  /**
   * @brief Initialize geometry from observations
   * @param observations Vector of target observations
   * @return True if initialization succeeded
   */
  bool initGeometryFromObservations(
      const std::vector<aslam::cameras::GridCalibrationTargetObservation>&
          observations);

  // Accessors
  const ImageDatasetReader& getDataset() const { return dataset_; }
  std::shared_ptr<AslamCamera> getCamera() const { return camera_; }
  std::shared_ptr<aslam::cameras::CameraGeometryBase> getGeometry() const {
    return geometry_;
  }
  std::shared_ptr<TargetDetector> getTargetDetector() const { return ctarget_; }
  bool isGeometryInitialized() const { return isGeometryInitialized_; }
  const CameraParameters& getCameraModel() const { return cameraModel_; }

  /**
   * @brief Get projection design variable
   * This is a type-erased interface that works with any camera type
   */
  std::shared_ptr<aslam::backend::DesignVariable> getProjectionDv() const {
    return projectionDv_;
  }

  /**
   * @brief Get distortion design variable
   */
  std::shared_ptr<aslam::backend::DesignVariable> getDistortionDv() const {
    return distortionDv_;
  }

  /**
   * @brief Get shutter design variable
   */
  std::shared_ptr<aslam::backend::DesignVariable> getShutterDv() const {
    return shutterDv_;
  }

  /**
   * @brief Create reprojection error term
   * @param y Keypoint measurement
   * @param invR Inverse covariance matrix
   * @param p_c Point in camera frame (homogeneous expression)
   * @return Reprojection error term
   */
  std::shared_ptr<aslam::backend::ErrorTerm> createReprojectionError(
      const Eigen::VectorXd& y, const Eigen::MatrixXd& invR,
      const aslam::backend::HomogeneousExpression& p_c) const;

 private:
  // Initialize design variables based on camera type
  void initializeDesignVariables();

  ImageDatasetReader dataset_;
  CameraParameters cameraModel_;
  std::shared_ptr<AslamCamera> camera_;
  std::shared_ptr<aslam::cameras::CameraGeometryBase> geometry_;
  std::shared_ptr<TargetDetector> ctarget_;
  bool isGeometryInitialized_;

  // Type-erased design variables (set by initializeDesignVariables)
  std::shared_ptr<aslam::backend::DesignVariable> projectionDv_;
  std::shared_ptr<aslam::backend::DesignVariable> distortionDv_;
  std::shared_ptr<aslam::backend::DesignVariable> shutterDv_;

  // Store the actual typed camera design variable container
  // This is an opaque pointer to avoid exposing template types in header
  std::shared_ptr<void> cameraDvContainer_;

  // Factory function for creating ReprojectionError
  // This function pointer is set based on the actual camera type during
  // initialization and returns a ReprojectionError with correct template type
  using ReprojErrorFactory =
      std::function<std::shared_ptr<aslam::backend::ErrorTerm>(
          const Eigen::VectorXd&, const Eigen::MatrixXd&,
          const aslam::backend::HomogeneousExpression&, const void*)>;
  ReprojErrorFactory reprojErrorFactory_;
};

/**
 * @brief Calibration target with design variables for target points
 *
 * Wraps target points as HomogeneousPointDv for optimization
 */
class CalibrationTarget {
 public:
  /**
   * @brief Construct calibration target
   * @param target Grid calibration target
   * @param estimateLandmarks Whether to optimize landmark positions
   */
  explicit CalibrationTarget(
      const std::shared_ptr<aslam::cameras::GridCalibrationTargetBase>& target,
      bool estimateLandmarks = false);

  /**
   * @brief Get point expression for optimization
   * @param i Point index
   * @return Homogeneous point expression
   */
  aslam::backend::HomogeneousExpression getPointExpression(size_t i) const;

  /**
   * @brief Get number of points
   */
  size_t size() const { return P_t_dv_.size(); }

  /**
   * @brief Get point design variable
   * @param i Point index
   */
  std::shared_ptr<aslam::backend::HomogeneousPoint> getPointDv(size_t i) const {
    return P_t_dv_.at(i);
  }

  /**
   * @brief Get underlying target
   */
  std::shared_ptr<aslam::cameras::GridCalibrationTargetBase> getTarget() const {
    return target_;
  }

 private:
  std::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target_;
  std::vector<std::shared_ptr<aslam::backend::HomogeneousPoint>> P_t_dv_;
  std::vector<aslam::backend::HomogeneousExpression> P_t_ex_;
};

// Forward declaration
class CameraCalibration;

/**
 * @brief Optimization problem for a single target view
 *
 * Contains all design variables and error terms for one observation
 */
class CalibrationTargetOptimizationProblem
    : public aslam::calibration::OptimizationProblem {
 public:
  /**
   * @brief Observation from a single camera in a rig
   */
  using RigObservation =
      std::pair<size_t, aslam::cameras::GridCalibrationTargetObservation>;

  /**
   * @brief Create optimization problem from observations
   * @param cameras Camera geometries
   * @param target Calibration target with DVs
   * @param baselines Baseline transformation DVs
   * @param timestamp Observation timestamp
   * @param T_tc_guess Initial target-to-camera0 transformation
   * @param rigObservations Observations from each camera
   * @param useBlakeZissermanMest Use Blake-Zisserman M-estimator
   * @return Optimization problem instance
   */
  static std::shared_ptr<CalibrationTargetOptimizationProblem>
  fromTargetViewObservations(
      const std::vector<std::shared_ptr<CameraGeometry>>& cameras,
      const std::shared_ptr<CalibrationTarget>& target,
      const std::vector<
          std::shared_ptr<aslam::backend::TransformationDesignVariable>>&
          baselines,
      const aslam::Time& timestamp,
      const sm::kinematics::Transformation& T_tc_guess,
      const std::vector<RigObservation>& rigObservations,
      bool useBlakeZissermanMest = true);

  /**
   * @brief Get reprojection errors for a camera
   * @param camId Camera ID
   * @return Vector of error terms (nullptr for invalid points)
   */
  const std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>&
  getReprojectionErrors(size_t camId) const;

  // Data stored for potential problem rebuilding
  std::vector<std::shared_ptr<CameraGeometry>> cameras;
  std::shared_ptr<CalibrationTarget> target;
  std::vector<std::shared_ptr<aslam::backend::TransformationDesignVariable>>
      baselines;
  aslam::Time timestamp;
  sm::kinematics::Transformation T_tc_guess;
  std::vector<RigObservation> rig_observations;

  // Target-to-camera transformation DV
  std::shared_ptr<aslam::backend::TransformationDesignVariable>
      dv_T_target_camera;

  // Reprojection errors per camera (indexed by corner)
  std::unordered_map<size_t,
                     std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>>
      rerrs;

 private:
  CalibrationTargetOptimizationProblem() = default;
};

/**
 * @brief Remove corners from a batch problem
 * @param batch Original batch problem
 * @param camIdCornerIdPairs List of (camera_id, corner_ids_to_remove) pairs
 * @param useBlakeZissermanMest Use Blake-Zisserman M-estimator in rebuilt
 * problem
 * @return New optimization problem with corners removed
 */
std::shared_ptr<CalibrationTargetOptimizationProblem> removeCornersFromBatch(
    const std::shared_ptr<CalibrationTargetOptimizationProblem>& batch,
    const std::vector<std::pair<size_t, std::vector<size_t>>>&
        camIdCornerIdPairs,
    bool useBlakeZissermanMest = true);

/**
 * @brief Main camera calibration controller
 *
 * Manages incremental calibration with multiple cameras
 */
class CameraCalibration {
 public:
  /**
   * @brief Construct camera calibration
   * @param cameras Camera geometry wrappers
   * @param baselineGuesses Initial baseline transformations
   * @param estimateLandmarks Optimize target point positions
   * @param verbose Enable verbose output
   * @param useBlakeZissermanMest Use Blake-Zisserman M-estimator
   */
  CameraCalibration(
      const std::vector<std::shared_ptr<CameraGeometry>>& cameras,
      const std::vector<sm::kinematics::Transformation>& baselineGuesses,
      bool estimateLandmarks = false, bool verbose = false,
      bool useBlakeZissermanMest = true);

  /**
   * @brief Add a target view to the calibration
   * @param timestamp View timestamp
   * @param rigObservations Observations from each camera
   * @param T_tc_guess Initial target-to-camera transformation
   * @param force Force addition even if information gain is low
   * @return True if view was accepted by estimator
   */
  bool addTargetView(
      const aslam::Time& timestamp,
      const std::vector<CalibrationTargetOptimizationProblem::RigObservation>&
          rigObservations,
      const sm::kinematics::Transformation& T_tc_guess, bool force = false);

  /**
   * @brief Get baseline transformation DV
   * @param i Baseline index (0 = cam0->cam1, etc.)
   */
  std::shared_ptr<aslam::backend::TransformationDesignVariable> getBaseline(
      size_t i) const {
    return baselines_.at(i);
  }

  /**
   * @brief Get all baselines
   */
  const std::vector<
      std::shared_ptr<aslam::backend::TransformationDesignVariable>>&
  getBaselines() const {
    return baselines_;
  }

  /**
   * @brief Get cameras
   */
  const std::vector<std::shared_ptr<CameraGeometry>>& getCameras() const {
    return cameras_;
  }

  /**
   * @brief Get views
   */
  const std::vector<std::shared_ptr<CalibrationTargetOptimizationProblem>>&
  getViews() const {
    return views_;
  }

  /**
   * @brief Get calibration target
   */
  std::shared_ptr<CalibrationTarget> getTarget() const { return target_; }

  /**
   * @brief Get number of cameras
   */
  size_t numCameras() const { return cameras_.size(); }

  /**
   * @brief Get number of views
   */
  size_t numViews() const { return views_.size(); }

 private:
  void initializeBaselineDVs(
      const std::vector<sm::kinematics::Transformation>& baselineGuesses);

  std::vector<std::shared_ptr<CameraGeometry>> cameras_;
  std::shared_ptr<CalibrationTarget> target_;
  std::vector<std::shared_ptr<aslam::backend::TransformationDesignVariable>>
      baselines_;
  std::vector<std::shared_ptr<CalibrationTargetOptimizationProblem>> views_;

  bool useBlakeZissermanMest_;
  aslam::calibration::IncrementalEstimator estimator_;
  aslam::calibration::LinearSolverOptions linearSolverOptions_;
  aslam::backend::Optimizer2Options optimizerOptions_;
  aslam::calibration::IncrementalEstimator::ReturnValue estimatorReturnValue_;
};

}  // namespace kalibr

#endif  // KALIBR_CAMERA_CALIBRATOR_HPP
