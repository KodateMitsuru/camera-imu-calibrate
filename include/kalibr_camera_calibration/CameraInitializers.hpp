#ifndef KALIBR_CAMERA_CALIBRATION_CAMERA_INITIALIZERS_HPP
#define KALIBR_CAMERA_CALIBRATION_CAMERA_INITIALIZERS_HPP

#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <memory>
#include <optional>
#include <sm/kinematics/Transformation.hpp>
#include <utility>
#include <vector>

#include "CameraCalibrator.hpp"

namespace kalibr {

// Forward declarations
class MulticamGraph;

/**
 * @brief Add a pose design variable to an optimization problem
 *
 * Creates rotation (quaternion) and translation (euclidean point) design
 * variables and adds them to the optimization problem.
 *
 * @param problem The optimization problem to add variables to
 * @param T0 Initial transformation guess (default: identity)
 * @return shared_ptr to TransformationBasic containing the created design
 * variables
 */
std::shared_ptr<aslam::backend::TransformationBasic> addPoseDesignVariable(
    aslam::backend::OptimizationProblem& problem,
    const sm::kinematics::Transformation& T0 =
        sm::kinematics::Transformation());

/**
 * @brief Stereo camera calibration result
 */
struct StereoCalibrateResult {
  bool success = false;
  sm::kinematics::Transformation baseline_HL;  // Transform from cam_L to cam_H
};

/**
 * @brief Calibrate stereo camera baseline using bundle adjustment
 *
 * Finds the rigid body transformation between two cameras by jointly
 * optimizing the baseline and target poses using reprojection errors.
 *
 * @param camL_geometry Left camera geometry with model and design variables
 * @param camH_geometry Right camera geometry with model and design variables
 * @param obslist List of (left_obs, right_obs) observation pairs. Each element
 *                may be nullopt if camera didn't see target in that view.
 * @param distortionActive Whether to optimize distortion parameters
 * @param baseline Optional initial baseline guess. If not provided, computed
 *                 from median of PnP solutions.
 * @return StereoCalibrateResult with success flag and optimized baseline
 */
StereoCalibrateResult stereoCalibrate(
    CameraGeometry& camL_geometry, CameraGeometry& camH_geometry,
    const std::vector<std::pair<
        std::optional<aslam::cameras::GridCalibrationTargetObservation>,
        std::optional<aslam::cameras::GridCalibrationTargetObservation>>>&
        obslist,
    bool distortionActive = false,
    const std::optional<sm::kinematics::Transformation>& baseline =
        std::nullopt);

/**
 * @brief Calibrate single camera intrinsics using bundle adjustment
 *
 * Optimizes camera intrinsic parameters (and optionally distortion) by
 * minimizing reprojection errors across all calibration target views.
 *
 * @param cam_geometry Camera geometry with model and design variables
 * @param obslist List of calibration target observations
 * @param distortionActive Whether to optimize distortion parameters
 * @param intrinsicsActive Whether to optimize intrinsic parameters
 * @return true if optimization succeeded, false otherwise
 */
bool calibrateIntrinsics(
    CameraGeometry& cam_geometry,
    const std::vector<aslam::cameras::GridCalibrationTargetObservation>&
        obslist,
    bool distortionActive = true, bool intrinsicsActive = true);

/**
 * @brief Full batch optimization result
 */
struct FullBatchResult {
  bool success = false;
  std::vector<sm::kinematics::Transformation> baselines;
};

/**
 * @brief Solve full batch calibration for camera chain
 *
 * Jointly optimizes all camera intrinsics, distortion parameters, and
 * inter-camera baselines using all available observations.
 *
 * @param cameras Vector of camera geometries in the chain
 * @param baseline_guesses Initial guesses for inter-camera baselines
 * @param graph Multicam observation graph containing all observations
 * @return FullBatchResult with success flag and optimized baselines
 */
FullBatchResult solveFullBatch(
    std::vector<CameraGeometry*>& cameras,
    const std::vector<sm::kinematics::Transformation>& baseline_guesses,
    MulticamGraph& graph);

// ============================================================================
// Helper functions
// ============================================================================

/**
 * @brief Create optimizer options for camera calibration
 *
 * @param verbose Enable verbose output
 * @param maxIterations Maximum optimization iterations
 * @return Configured Optimizer2Options
 */
aslam::backend::Optimizer2Options createCameraOptimizerOptions(
    bool verbose = false, int maxIterations = 200);

/**
 * @brief Compute reprojection error statistics for a set of error terms
 *
 * @param errors Vector of error term pointers
 * @return tuple of (mean, median, std) of squared errors
 */
std::tuple<double, double, double> computeReprojectionErrorStats(
    const std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>& errors);

}  // namespace kalibr

#endif  // KALIBR_CAMERA_CALIBRATION_CAMERA_INITIALIZERS_HPP
